#include <gtest/gtest.h>

#include <algorithm>
#include <functional>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <limits>
#include <exception>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "fdp_test_utils.h"
#include "option/option.h"
#include "preprocess/pass/feasible_domain_propagator/fdp_utility.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_arithmetic.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_boolean.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_comparison.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_composition.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_division.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_ite.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_multiplication.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_shifting.h"
#include "printer/printer.h"
#include "solving_context.h"
#include "util/logger.h"

namespace bzla::preprocess::pass::fdp {
namespace {

using ::bzla::BitVector;
using ::bzla::Node;
using ::bzla::Type;

namespace fs = std::filesystem;

constexpr const char* kOutputDir = "dev-test-output";
constexpr const char* kSmtOutputDir = "dev-test-output/smt2";
constexpr const char* kLogFilePath = "dev-test-output/fdp_operator_soundness.log";
constexpr const char* kErrorLogFilePath = "dev-test-output/fdp_operator_soundness_error.log";

thread_local std::ostringstream* g_attempt_log_stream = nullptr;
thread_local bool g_attempt_log_has_error = false;

void EnsureOutputDirectories() {
    static const bool created = []() {
        std::error_code ec;
        fs::create_directories(kOutputDir, ec);
        fs::create_directories(kSmtOutputDir, ec);
        return true;
    }();
    (void) created;
}

std::ofstream& LogFile() {
    static std::ofstream file;
    if (!file.is_open()) {
        EnsureOutputDirectories();
        file.open(kLogFilePath, std::ios::out | std::ios::app);
    }
    return file;
}

std::ofstream& ErrorLogFile() {
    static std::ofstream file;
    if (!file.is_open()) {
        EnsureOutputDirectories();
        file.open(kErrorLogFilePath, std::ios::out | std::ios::app);
    }
    return file;
}

void EmitLog(const std::string& text) {
    std::cout << text;
    std::ofstream& file = LogFile();
    if (file) {
        file << text;
    }
    if (g_attempt_log_stream) {
        *g_attempt_log_stream << text;
    }
}

void EmitError(const std::string& text) {
    std::ofstream& file = ErrorLogFile();
    if (file) {
        file << text;
    }
    if (g_attempt_log_stream) {
        g_attempt_log_has_error = true;
    }
}

void EmitAttemptErrorLog() {
    if (g_attempt_log_stream) {
        g_attempt_log_has_error = true;
    }
}

class AttemptLogScope {
 public:
    AttemptLogScope()
        : d_prev_stream(g_attempt_log_stream), d_prev_has_error(g_attempt_log_has_error) {
        g_attempt_log_stream = &d_stream;
        g_attempt_log_has_error = false;
    }

    ~AttemptLogScope() {
        if (g_attempt_log_stream == &d_stream && g_attempt_log_has_error) {
            EmitError("[fdp-error-log-copy] begin\n");
            EmitError(d_stream.str());
            EmitError("[fdp-error-log-copy] end\n");
        }
        g_attempt_log_stream = d_prev_stream;
        g_attempt_log_has_error = d_prev_has_error;
    }

 private:
    std::ostringstream d_stream;
    std::ostringstream* d_prev_stream;
    bool d_prev_has_error;
};

std::string ValueToString(const Node& term, const Node& value) {
    if (value.is_null()) {
        return "<null>";
    }
    if (term.type().is_bool()) {
        return value.value<bool>() ? "true" : "false";
    }
    if (term.type().is_bv()) {
        return value.value<BitVector>().str(2);
    }
    return value.str();
}

std::string TargetLabel(size_t target, size_t num_children) {
    if (target == std::numeric_limits<size_t>::max()) {
        return "all";
    }
    return target == num_children ? "output" : "child[" + std::to_string(target) + "]";
}

bool DomainsEqual(const FeasibleDomain& lhs, const FeasibleDomain& rhs) {
    if (lhs.width() != rhs.width()) {
        return false;
    }
    if (lhs.interval_min() != rhs.interval_min() || lhs.interval_max() != rhs.interval_max()
        || lhs.is_interval_complementary() != rhs.is_interval_complementary()) {
        return false;
    }
    for (uint32_t bit = 0; bit < lhs.width(); ++bit) {
        if (lhs.is_fixed(bit) != rhs.is_fixed(bit)) {
            return false;
        }
        if (lhs.is_fixed(bit) && lhs.get_value(bit) != rhs.get_value(bit)) {
            return false;
        }
    }
    return true;
}
struct OperatorSample {
    OperatorSample() : output(1) {}

    std::vector<FeasibleDomain> children;
    std::vector<Type> child_types;
    std::vector<bool> check_domains;
    FeasibleDomain output;
    Type output_type;
    std::vector<uint64_t> indices;
};

struct OperatorSpec {
    const char* name;
    node::Kind kind;
    uint32_t arity;
    bool children_bool;
    bool output_bool;
    uint32_t min_width;
    uint32_t max_width;
    std::function<std::unique_ptr<FdpOperator>(FdpOperator::DomainVector&,
                                               FeasibleDomain*,
                                               const OperatorSample&)> factory;
    std::function<std::optional<OperatorSample>(NodeManager&,
                                                std::mt19937&,
                                                const OperatorSpec&)> sample = nullptr;
    std::function<Node(NodeManager&, const OperatorSample&, const std::vector<Node>&)> term_builder = nullptr;
};

std::vector<uint64_t> EnumerateValues(const FeasibleDomain& domain, bool negate) {
    std::vector<uint64_t> values;
    const uint64_t upper = uint64_t{1} << domain.width();
    for (uint64_t v = 0; v < upper; ++v) {
        const bool holds = feasible_domain_holds_unsigned(domain, static_cast<uint32_t>(v));
        if ((holds && !negate) || (!holds && negate)) {
            values.push_back(v);
        }
    }
    return values;
}

struct EvalResult {
    bool is_bool = false;
    uint64_t value = 0;
};

EvalResult EvaluateOperator(const OperatorSpec& spec,
                            const OperatorSample& sample,
                            const std::vector<uint64_t>& child_values) {
    EvalResult res;
    switch (spec.kind) {
        case node::Kind::BV_ADD: {
            BitVector bv = test::BvFromUint(sample.output.width(), child_values[0]);
            bv = bv.bvadd(test::BvFromUint(sample.output.width(), child_values[1]));
            res.value = bv.to_uint64();
            break;
        }
        case node::Kind::BV_MUL: {
            BitVector bv = test::BvFromUint(sample.output.width(), child_values[0]);
            bv = bv.bvmul(test::BvFromUint(sample.output.width(), child_values[1]));
            res.value = bv.to_uint64();
            break;
        }
        case node::Kind::BV_UDIV: {
            BitVector bv = test::BvFromUint(sample.output.width(), child_values[0]);
            bv = bv.bvudiv(test::BvFromUint(sample.output.width(), child_values[1]));
            res.value = bv.to_uint64();
            break;
        }
        case node::Kind::BV_UREM: {
            BitVector bv = test::BvFromUint(sample.output.width(), child_values[0]);
            bv = bv.bvurem(test::BvFromUint(sample.output.width(), child_values[1]));
            res.value = bv.to_uint64();
            break;
        }
        case node::Kind::BV_SHL: {
            BitVector bv = test::BvFromUint(sample.output.width(), child_values[0]);
            bv = bv.bvshl(test::BvFromUint(sample.children[1].width(), child_values[1]));
            res.value = bv.to_uint64();
            break;
        }
        case node::Kind::BV_SHR: {
            BitVector bv = test::BvFromUint(sample.output.width(), child_values[0]);
            bv = bv.bvshr(test::BvFromUint(sample.children[1].width(), child_values[1]));
            res.value = bv.to_uint64();
            break;
        }
        case node::Kind::BV_ASHR: {
            BitVector bv = test::BvFromUint(sample.output.width(), child_values[0]);
            bv = bv.bvashr(test::BvFromUint(sample.children[1].width(), child_values[1]));
            res.value = bv.to_uint64();
            break;
        }
        case node::Kind::BV_AND: {
            BitVector bv = test::BvFromUint(sample.output.width(), child_values[0]);
            bv = bv.bvand(test::BvFromUint(sample.output.width(), child_values[1]));
            res.value = bv.to_uint64();
            break;
        }
        case node::Kind::BV_XOR: {
            BitVector bv = test::BvFromUint(sample.output.width(), child_values[0]);
            bv = bv.bvxor(test::BvFromUint(sample.output.width(), child_values[1]));
            res.value = bv.to_uint64();
            break;
        }
        case node::Kind::AND: {
            res.is_bool = true;
            res.value = (child_values[0] && child_values[1]) ? 1 : 0;
            break;
        }
        case node::Kind::XOR: {
            res.is_bool = true;
            res.value = (child_values[0] ^ child_values[1]) ? 1 : 0;
            break;
        }
        case node::Kind::NOT: {
            res.is_bool = true;
            res.value = child_values[0] ? 0 : 1;
            break;
        }
        case node::Kind::EQUAL: {
            res.is_bool = true;
            res.value = (child_values[0] == child_values[1]) ? 1 : 0;
            break;
        }
        case node::Kind::BV_ULT: {
            BitVector bv0 = test::BvFromUint(sample.children[0].width(), child_values[0]);
            BitVector bv1 = test::BvFromUint(sample.children[1].width(), child_values[1]);
            res.is_bool = true;
            res.value = bv0.bvult(bv1).is_one() ? 1 : 0;
            break;
        }
        case node::Kind::BV_SLT: {
            BitVector bv0 = test::BvFromUint(sample.children[0].width(), child_values[0]);
            BitVector bv1 = test::BvFromUint(sample.children[1].width(), child_values[1]);
            res.is_bool = true;
            res.value = bv0.bvslt(bv1).is_one() ? 1 : 0;
            break;
        }
        case node::Kind::BV_CONCAT: {
            BitVector high = test::BvFromUint(sample.children[0].width(), child_values[0]);
            BitVector low = test::BvFromUint(sample.children[1].width(), child_values[1]);
            res.value = high.bvconcat(low).to_uint64();
            break;
        }
        case node::Kind::BV_EXTRACT: {
            BitVector bv = test::BvFromUint(sample.children[0].width(), child_values[0]);
            const uint64_t high = sample.indices[0];
            const uint64_t low = sample.indices[1];
            res.value = bv.bvextract(high, low).to_uint64();
            break;
        }
        case node::Kind::ITE: {
            res.value = child_values[0] ? child_values[1] : child_values[2];
            res.is_bool = sample.child_types[1].is_bool();
            break;
        }
        default:
            break;
    }
    return res;
}

void LogPre(const OperatorSpec& spec,
            int attempt,
            const std::vector<FeasibleDomain>& before_children,
            const FeasibleDomain& before_output) {
    std::ostringstream oss;
    oss << "[fdp-pre] op=" << spec.name << " attempt=" << attempt << "\n";
    for (size_t idx = 0; idx < before_children.size(); ++idx) {
        oss << "  before child[" << idx << "]: " << before_children[idx].to_string(true) << "\n";
    }
    oss << "  before output: " << before_output.to_string(true) << "\n";
    EmitLog(oss.str());
}

void LogPost(const OperatorSpec& spec,
             int attempt,
             const OperatorSample& after_sample) {
    std::ostringstream oss;
    oss << "[fdp-post] op=" << spec.name << " attempt=" << attempt << "\n";
    for (size_t idx = 0; idx < after_sample.children.size(); ++idx) {
        oss << "  after child[" << idx << "]: " << after_sample.children[idx].to_string(true)
            << "\n";
    }
    oss << "  after output: " << after_sample.output.to_string(true) << "\n";
    EmitLog(oss.str());
}

constexpr uint64_t kSolveTimeoutMs = 2000;

struct ImplicationResult {
    bzla::Result result = bzla::Result::UNKNOWN;
    std::string counterexample;
    std::string smt_path;
};

std::string ValueToString(const Type& type, uint64_t value) {
    if (type.is_bool()) {
        return value ? "true" : "false";
    }
    return BitVector::from_ui(type.bv_size(), value).str(2);
}

std::string DumpSmt(const OperatorSpec& spec,
                    int attempt,
                    size_t target,
                    const char* phase,
                    const std::vector<Node>& assertions) {
    std::string filename = "fdp_" + std::string(spec.name) + "_" + phase + "_"
                           + std::to_string(attempt);
    if (target != std::numeric_limits<size_t>::max()) {
        filename += "_" + std::to_string(target);
    }
    filename += ".smt2";

    EnsureOutputDirectories();
    fs::path smt_path = fs::path(kSmtOutputDir) / filename;
    std::ofstream out(smt_path);
    if (!out) {
        EmitError("[fdp-error] failed to open smt dump file: " + smt_path.string() + "\n");
        EmitAttemptErrorLog();
        return "";
    }

    Printer::print_formula(out, assertions);
    return smt_path.string();
}

Node EncodeDomainConjunction(NodeManager& nm,
                             const std::vector<Node>& terms,
                             const std::vector<FeasibleDomain>& domains) {
    std::vector<Node> parts;
    parts.reserve(terms.size());
    for (size_t idx = 0; idx < terms.size(); ++idx) {
        Node encoded = test::EncodeFeasibleDomain(nm, terms[idx], domains[idx]);
        if (!test::IsTrueNode(encoded)) {
            parts.push_back(encoded);
        }
    }
    if (parts.empty()) {
        return nm.mk_value(true);
    }
    if (parts.size() == 1) {
        return parts[0];
    }
    // Kind::AND is binary in the node manager, fold manually.
    Node conj = parts[0];
    for (size_t i = 1; i < parts.size(); ++i) {
        conj = nm.mk_node(node::Kind::AND, {conj, parts[i]});
    }
    return conj;
}

void LogResult(const OperatorSpec& spec,
               size_t num_children,
               size_t target,
               const ImplicationResult& implication) {
    std::ostringstream oss;
    oss << "[fdp-result] op=" << spec.name << " target=" << TargetLabel(target, num_children)
        << " status=";
    if (implication.result == bzla::Result::UNSAT) {
        oss << "PASS";
    }
    else if (implication.result == bzla::Result::SAT) {
        oss << "FAIL (sat)";
    }
    else {
        oss << "FAIL (unknown)";
    }
    if (!implication.smt_path.empty()) {
        oss << " smt=" << implication.smt_path;
    }
    if (!implication.counterexample.empty()) {
        oss << "\n" << implication.counterexample;
    }
    oss << "\n";
    EmitLog(oss.str());
    if (implication.result != bzla::Result::UNSAT) {
        EmitError(oss.str());
        EmitAttemptErrorLog();
    }
}

void LogCrash(const OperatorSpec& spec,
              const std::string& target_label,
              int attempt,
              const std::string& smt_path,
              const std::string& message) {
    std::ostringstream oss;
    oss << "[fdp-crash] op=" << spec.name << " target=" << target_label
        << " attempt=" << attempt << " reason=" << message;
    if (!smt_path.empty()) {
        oss << " smt=" << smt_path;
    }
    oss << "\n";
    EmitLog(oss.str());
    EmitError(oss.str());
    EmitAttemptErrorLog();
}

template <typename Solver>
bzla::Result RunSolveWithCatcher(Solver&& solver, std::string* crash_reason) {
    try {
        return solver();
    }
    catch (const std::exception& ex) {
        if (crash_reason) {
            *crash_reason = ex.what();
        }
    }
    catch (...) {
        if (crash_reason) {
            *crash_reason = "unknown non-std exception";
        }
    }
    return bzla::Result::UNKNOWN;
}

class FdpOperatorSoundnessTest : public ::testing::TestWithParam<OperatorSpec> {
 protected:
    FdpOperatorSoundnessTest() {
        d_options.set(option::Option::BV_SOLVER, std::string("bitblast"));
    }

    uint32_t pick_width(const OperatorSpec& spec, std::mt19937& rng) {
        if (spec.children_bool) {
            return 1;
        }
        std::uniform_int_distribution<uint32_t> dist(spec.min_width, spec.max_width);
        return dist(rng);
    }

    OperatorSample make_standard_sample(const OperatorSpec& spec, std::mt19937& rng) {
        const uint32_t width = pick_width(spec, rng);
        OperatorSample sample;
        sample.children.reserve(spec.arity);
        sample.child_types.reserve(spec.arity);
        sample.check_domains.resize(spec.arity, true);

        Type child_type = spec.children_bool ? d_nm.mk_bool_type() : d_nm.mk_bv_type(width);
        Type output_type = spec.output_bool ? d_nm.mk_bool_type() : d_nm.mk_bv_type(width);
        for (uint32_t idx = 0; idx < spec.arity; ++idx) {
            sample.children.emplace_back(
                test::RandomDomain(rng, spec.children_bool ? 1u : width, spec.children_bool));
            sample.child_types.push_back(child_type);
        }
        sample.output =
            test::RandomDomain(rng, output_type.is_bool() ? 1u : width, spec.output_bool);
        sample.output_type = output_type;
        return sample;
    }

    std::optional<OperatorSample> make_sample(const OperatorSpec& spec, std::mt19937& rng) {
        if (spec.sample) {
            return spec.sample(d_nm, rng, spec);
        }
        return make_standard_sample(spec, rng);
    }

    ImplicationResult check_implication(const OperatorSpec& spec,
                                        const std::vector<FeasibleDomain>& pre_children,
                                        const FeasibleDomain& pre_output,
                                        const OperatorSample& post_sample,
                                        const std::vector<Node>& child_terms,
                                        const std::vector<std::string>& child_names,
                                        const Node& output_term,
                                        const Node& op_term,
                                        int attempt) {
        option::Options opts = d_options;
        opts.set(option::Option::TIME_LIMIT_PER, kSolveTimeoutMs);
        opts.set(option::Option::PP_FDP_ENABLE, false);
        SolvingContext ctx(d_nm, opts);
        std::vector<Node> assertions;
        auto add_assert = [&](const Node& n) {
            ctx.assert_formula(n);
            assertions.push_back(n);
        };

        // Verify (FD_pre(children) and FD_pre(output) and equality) implies FD_post(all).
        for (size_t idx = 0; idx < pre_children.size(); ++idx) {
            Node fd = test::EncodeFeasibleDomain(d_nm, child_terms[idx], pre_children[idx]);
            if (!test::IsTrueNode(fd)) {
                add_assert(fd);
            }
        }

        Node pre_fd_out = test::EncodeFeasibleDomain(d_nm, output_term, pre_output);
        if (!test::IsTrueNode(pre_fd_out)) {
            add_assert(pre_fd_out);
        }

        add_assert(d_nm.mk_node(node::Kind::EQUAL, {output_term, op_term}));

        std::vector<Node> post_terms(child_terms);
        post_terms.push_back(output_term);
        std::vector<FeasibleDomain> post_domains(post_sample.children.begin(),
                                                 post_sample.children.end());
        post_domains.push_back(post_sample.output);
        Node post_conj = EncodeDomainConjunction(d_nm, post_terms, post_domains);

        ImplicationResult implication;
        if (test::IsTrueNode(post_conj)) {
            implication.result = bzla::Result::UNSAT;
            return implication;
        }

        add_assert(d_nm.mk_node(node::Kind::NOT, {post_conj}));

        implication.smt_path =
            DumpSmt(spec, attempt, std::numeric_limits<size_t>::max(), "imp", assertions);
        if (!implication.smt_path.empty()) {
            std::ostringstream oss;
            oss << "[fdp-smt] op=" << spec.name << " attempt=" << attempt
                << " target=" << TargetLabel(std::numeric_limits<size_t>::max(),
                                             post_sample.children.size())
                << " file=" << implication.smt_path << "\n";
            EmitLog(oss.str());
        }

        auto quick_check = [&]() -> std::optional<ImplicationResult> {
            std::vector<std::vector<uint64_t>> value_sets;
            value_sets.reserve(pre_children.size());
            for (const auto& dom : pre_children) {
                value_sets.push_back(EnumerateValues(dom, false));
                if (value_sets.back().empty()) {
                    ImplicationResult result;
                    result.result = bzla::Result::UNSAT;
                    return result;
                }
            }

            uint64_t total = 1;
            for (const auto& vs : value_sets) {
                total *= static_cast<uint64_t>(vs.size());
                if (total > (1u << 20)) {
                    return std::nullopt;
                }
            }

            std::vector<size_t> indices(pre_children.size(), 0);
            bool done = false;
            while (!done) {
                std::vector<uint64_t> values;
                values.reserve(pre_children.size());
                for (size_t i = 0; i < pre_children.size(); ++i) {
                    values.push_back(value_sets[i][indices[i]]);
                }

                EvalResult eval = EvaluateOperator(spec, post_sample, values);
                const uint64_t out_value = eval.value;

                if (feasible_domain_holds_unsigned(pre_output,
                                                   static_cast<uint32_t>(out_value))) {
                    bool post_ok =
                        feasible_domain_holds_unsigned(post_sample.output,
                                                       static_cast<uint32_t>(out_value));
                    std::vector<std::string> violated;
                    if (!post_ok) {
                        violated.push_back("output");
                    }
                    for (size_t i = 0; i < values.size(); ++i) {
                        if (!feasible_domain_holds_unsigned(post_sample.children[i],
                                                            static_cast<uint32_t>(values[i]))) {
                            post_ok = false;
                            violated.push_back(child_names[i]);
                        }
                    }
                    if (!post_ok) {
                        ImplicationResult result;
                        result.result = bzla::Result::SAT;
                        std::ostringstream counterexample;
                        counterexample << "counterexample for " << spec.name << " all:\n";
                        for (size_t i = 0; i < values.size(); ++i) {
                            counterexample << "  " << child_names[i] << " = "
                                           << ValueToString(post_sample.child_types[i], values[i])
                                           << "\n";
                        }
                        counterexample << "  output = "
                                       << ValueToString(post_sample.output_type, out_value) << "\n";
                        if (!violated.empty()) {
                            counterexample << "  violated: ";
                            for (size_t i = 0; i < violated.size(); ++i) {
                                counterexample << violated[i];
                                if (i + 1 < violated.size()) {
                                    counterexample << ", ";
                                }
                            }
                            counterexample << "\n";
                        }
                        result.counterexample = counterexample.str();
                        return result;
                    }
                }

                size_t pos = pre_children.size();
                bool advanced = false;
                while (pos > 0) {
                    --pos;
                    if (++indices[pos] < value_sets[pos].size()) {
                        advanced = true;
                        break;
                    }
                    indices[pos] = 0;
                }
                if (!advanced) {
                    done = true;
                }
            }

            ImplicationResult result;
            result.result = bzla::Result::UNSAT;
            return result;
        }();

        if (quick_check) {
            quick_check->smt_path = implication.smt_path;
            return *quick_check;
        }

        std::string crash_reason;
        implication.result = RunSolveWithCatcher([&ctx]() { return ctx.solve(); }, &crash_reason);
        if (!crash_reason.empty()) {
            implication.counterexample = "solver crashed during implication check: " + crash_reason;
            LogCrash(spec,
                     TargetLabel(std::numeric_limits<size_t>::max(), post_sample.children.size()),
                     attempt,
                     implication.smt_path,
                     crash_reason);
            return implication;
        }
        if (implication.result == bzla::Result::SAT) {
            std::ostringstream counterexample;
            counterexample << "counterexample for " << spec.name << " all:\n";
            for (size_t idx = 0; idx < child_terms.size(); ++idx) {
                Node value = ctx.get_value(child_terms[idx]);
                counterexample << "  " << child_names[idx] << " = "
                               << ValueToString(child_terms[idx], value) << "\n";
            }
            Node out_value = ctx.get_value(output_term);
            counterexample << "  output = " << ValueToString(output_term, out_value) << "\n";
            implication.counterexample = counterexample.str();
        }
        else if (implication.result == bzla::Result::UNKNOWN) {
            implication.counterexample = "solver returned UNKNOWN";
        }

        return implication;
    }

    ImplicationResult check_original_unsat(const OperatorSpec& spec,
                                           const std::vector<FeasibleDomain>& children,
                                           const FeasibleDomain& output,
                                           const std::vector<Type>& child_types,
                                           const Type& output_type,
                                           const std::vector<uint64_t>& indices,
                                           int attempt) {
        option::Options opts = d_options;
        opts.set(option::Option::TIME_LIMIT_PER, kSolveTimeoutMs);
        opts.set(option::Option::PP_FDP_ENABLE, false);
        SolvingContext ctx(d_nm, opts);
        std::vector<Node> assertions;

        std::vector<std::string> child_names;
        std::vector<Node> child_terms;
        child_names.reserve(children.size());
        child_terms.reserve(children.size());
        for (size_t idx = 0; idx < children.size(); ++idx) {
            std::string name = std::string(spec.name) + "_conf_" + std::to_string(idx) + "_"
                               + std::to_string(attempt);
            child_names.push_back(name);
            child_terms.emplace_back(d_nm.mk_const(child_types[idx], name));
        }
        Node output_term = d_nm.mk_const(output_type,
                                         std::string(spec.name) + "_conf_out_"
                                             + std::to_string(attempt));

        auto add_assert = [&](const Node& n) {
            ctx.assert_formula(n);
            assertions.push_back(n);
        };

        for (size_t idx = 0; idx < children.size(); ++idx) {
            Node fd = test::EncodeFeasibleDomain(d_nm, child_terms[idx], children[idx]);
            if (!test::IsTrueNode(fd)) {
                add_assert(fd);
            }
        }

        Node fd_out = test::EncodeFeasibleDomain(d_nm, output_term, output);
        if (!test::IsTrueNode(fd_out)) {
            add_assert(fd_out);
        }

        OperatorSample tmp_sample;
        tmp_sample.children.reserve(children.size());
        for (const auto& dom : children) {
            tmp_sample.children.emplace_back(dom);
        }
        tmp_sample.child_types = child_types;
        tmp_sample.output_type = output_type;
        tmp_sample.indices = indices;

        Node op_term = spec.term_builder ? spec.term_builder(d_nm, tmp_sample, child_terms)
                                         : d_nm.mk_node(spec.kind, child_terms);
        add_assert(d_nm.mk_node(node::Kind::EQUAL, {output_term, op_term}));

        ImplicationResult implication;
        implication.smt_path =
            DumpSmt(spec, attempt, std::numeric_limits<size_t>::max(), "conflict", assertions);
        if (!implication.smt_path.empty()) {
            std::ostringstream oss;
            oss << "[fdp-smt] op=" << spec.name << " attempt=" << attempt
                << " target=conflict file=" << implication.smt_path << "\n";
            EmitLog(oss.str());
        }
        std::string crash_reason;
        implication.result = RunSolveWithCatcher([&ctx]() { return ctx.solve(); }, &crash_reason);
        if (!crash_reason.empty()) {
            implication.counterexample =
                "solver crashed during conflict confirmation: " + crash_reason;
            LogCrash(spec, "conflict", attempt, implication.smt_path, crash_reason);
            return implication;
        }
        if (implication.result == bzla::Result::SAT) {
            std::ostringstream counterexample;
            counterexample << "conflict reported but SAT for " << spec.name << ":\n";
            for (size_t idx = 0; idx < child_terms.size(); ++idx) {
                Node value = ctx.get_value(child_terms[idx]);
                counterexample << "  " << child_names[idx] << " = "
                               << ValueToString(child_terms[idx], value) << "\n";
            }
            Node out_value = ctx.get_value(output_term);
            counterexample << "  output = " << ValueToString(output_term, out_value) << "\n";
            implication.counterexample = counterexample.str();
        }
        else if (implication.result == bzla::Result::UNKNOWN) {
            implication.counterexample = "solver returned UNKNOWN";
        }

        return implication;
    }

    NodeManager d_nm;
    option::Options d_options;
};

namespace {

OperatorSample MakeConcatSample(NodeManager& nm,
                                std::mt19937& rng,
                                const OperatorSpec& spec) {
    OperatorSample sample;
    std::uniform_int_distribution<uint32_t> width_dist(spec.min_width, spec.max_width);
    const uint32_t high_width = width_dist(rng);
    const uint32_t low_width = width_dist(rng);

    sample.children.emplace_back(test::RandomDomain(rng, high_width, false));
    sample.children.emplace_back(test::RandomDomain(rng, low_width, false));
    sample.child_types = {nm.mk_bv_type(high_width), nm.mk_bv_type(low_width)};
    sample.check_domains = {true, true};

    sample.output = test::RandomDomain(rng, high_width + low_width, false);
    sample.output_type = nm.mk_bv_type(high_width + low_width);
    return sample;
}

OperatorSample MakeIteSample(NodeManager& nm,
                             std::mt19937& rng,
                             const OperatorSpec& spec) {
    const uint32_t width = std::uniform_int_distribution<uint32_t>(spec.min_width, spec.max_width)(rng);
    OperatorSample sample;

    sample.children.emplace_back(test::RandomDomain(rng, 1, true));
    sample.children.emplace_back(test::RandomDomain(rng, width, false));
    sample.children.emplace_back(test::RandomDomain(rng, width, false));
    sample.child_types = {nm.mk_bool_type(), nm.mk_bv_type(width), nm.mk_bv_type(width)};
    sample.check_domains = {true, true, true};

    sample.output = test::RandomDomain(rng, width, false);
    sample.output_type = nm.mk_bv_type(width);
    return sample;
}

OperatorSample MakeExtractSample(NodeManager& nm,
                                 std::mt19937& rng,
                                 const OperatorSpec& spec) {
    const uint32_t input_width = std::uniform_int_distribution<uint32_t>(
        std::max(3u, spec.min_width), std::max(3u, spec.max_width))(rng);
    std::uniform_int_distribution<uint32_t> low_dist(0, input_width - 1);
    const uint32_t low = low_dist(rng);
    std::uniform_int_distribution<uint32_t> high_dist(low, input_width - 1);
    const uint32_t high = high_dist(rng);

    OperatorSample sample;
    sample.children.emplace_back(test::RandomDomain(rng, input_width, false));
    sample.child_types = {nm.mk_bv_type(input_width)};
    sample.check_domains = {true};

    const uint32_t extract_width = high - low + 1;
    sample.output = test::RandomDomain(rng, extract_width, false);
    sample.output_type = nm.mk_bv_type(extract_width);
    sample.indices = {high, low};
    return sample;
}

}  // namespace

TEST_P(FdpOperatorSoundnessTest, RandomizedPropagationIsSound) {
    const OperatorSpec& spec = GetParam();
    std::mt19937 rng(static_cast<uint32_t>(std::hash<std::string>{}(spec.name)));

    constexpr int kRequiredImplications = 12;
    constexpr int kMaxAttempts = 200;
    int implications_checked = 0;

    for (int attempt = 0; attempt < kMaxAttempts && implications_checked < kRequiredImplications; ++attempt) {
        AttemptLogScope attempt_log_scope;
        auto sample_opt = make_sample(spec, rng);
        if (!sample_opt) {
            continue;
        }
        OperatorSample sample = std::move(*sample_opt);

        std::vector<FeasibleDomain> original_children = sample.children;
        FeasibleDomain original_output = sample.output;

        FdpOperator::DomainVector children_ptrs;
        children_ptrs.reserve(sample.children.size());
        for (auto& dom : sample.children) {
            children_ptrs.emplace_back(&dom);
        }

        {
            std::ostringstream oss;
            oss << "\n[fdp-new] op=" << spec.name << " attempt=" << attempt << "\n";
            EmitLog(oss.str());
        }
        LogPre(spec, attempt, original_children, original_output);

        std::unique_ptr<FdpOperator> op = spec.factory(children_ptrs, &sample.output, sample);
        std::string propagation_crash;
        Result res = Result::UNCHANGED;
        try {
            res = test::RunOperatorToFixedPoint(*op, children_ptrs, sample.output);
        }
        catch (const std::exception& ex) {
            propagation_crash = ex.what();
        }
        catch (...) {
            propagation_crash = "unknown non-std exception";
        }
        if (!propagation_crash.empty()) {
            LogCrash(spec, "propagation", attempt, "", propagation_crash);
            continue;
        }
        LogPost(spec, attempt, sample);
        if (is_conflict(res)) {
            ImplicationResult implication = check_original_unsat(spec,
                                                                 original_children,
                                                                 original_output,
                                                                 sample.child_types,
                                                                 sample.output_type,
                                                                 sample.indices,
                                                                 attempt);
            LogResult(spec, original_children.size(), original_children.size(), implication);
            if (implication.result == bzla::Result::SAT) {
                FAIL() << "operator returned CONFLICT but SMT is SAT\n"
                       << implication.counterexample;
            }
            continue;
        }
        for (auto& dom : sample.children) {
            test::ExpectInvariant(dom);
        }
        test::ExpectInvariant(sample.output);

        bool changed = false;
        for (size_t idx = 0; idx < sample.children.size(); ++idx) {
            if (!sample.check_domains[idx]) {
                continue;
            }
            if (!DomainsEqual(sample.children[idx], original_children[idx])) {
                changed = true;
                break;
            }
        }
        if (!changed && !DomainsEqual(sample.output, original_output)) {
            changed = true;
        }
        if (!changed) {
            continue;
        }

        std::vector<std::string> child_names;
        std::vector<Node> child_terms;
        child_names.reserve(sample.children.size());
        child_terms.reserve(sample.children.size());
        for (size_t idx = 0; idx < sample.children.size(); ++idx) {
            std::string name = std::string(spec.name) + "_" + std::to_string(idx) + "_"
                               + std::to_string(attempt);
            child_names.push_back(name);
            child_terms.emplace_back(d_nm.mk_const(sample.child_types[idx], name));
        }
        Node output_term = d_nm.mk_const(
            sample.output_type, std::string(spec.name) + "_out_" + std::to_string(attempt));
        Node op_term = spec.term_builder ? spec.term_builder(d_nm, sample, child_terms)
                                         : d_nm.mk_node(spec.kind, child_terms);

        SCOPED_TRACE(::testing::Message() << "op=" << spec.name << " attempt=" << attempt);
        ImplicationResult implication =
            check_implication(spec,
                              original_children,
                              original_output,
                              sample,
                              child_terms,
                              child_names,
                              output_term,
                              op_term,
                              attempt);
        LogResult(spec,
                  sample.children.size(),
                  std::numeric_limits<size_t>::max(),
                  implication);

        if (implication.result == bzla::Result::UNSAT) {
            ++implications_checked;
        }
        else if (implication.result == bzla::Result::SAT) {
            FAIL() << "found counterexample for " << spec.name << "\n"
                   << implication.counterexample;
        }
        // UNKNOWN or timeout: skip and continue with next attempt.
    }

    ASSERT_GE(implications_checked, kRequiredImplications)
        << "too few meaningful samples for " << spec.name;
}

static OperatorSpec kOperatorSpecs[] = {
    {"bv_add",
     node::Kind::BV_ADD,
     2,
     false,
     false,
     2,
     6,
     [](FdpOperator::DomainVector& children,
        FeasibleDomain* self,
        const OperatorSample& /*sample*/) {
         return std::make_unique<FdpAddOperator>(test::TestOperatorStats(), children, self);
     }},
    {"bv_mul",
     node::Kind::BV_MUL,
     2,
     false,
     false,
     2,
     6,
     [](FdpOperator::DomainVector& children,
        FeasibleDomain* self,
        const OperatorSample& /*sample*/) {
         return std::make_unique<FdpMulOperator>(test::TestOperatorStats(), children, self);
     }},
    {"bv_udiv",
     node::Kind::BV_UDIV,
     2,
     false,
     false,
     2,
     6,
     [](FdpOperator::DomainVector& children,
        FeasibleDomain* self,
        const OperatorSample& /*sample*/) {
         return std::make_unique<FdpDivOperator>(test::TestOperatorStats(), children, self);
     }},
    {"bv_urem",
     node::Kind::BV_UREM,
     2,
     false,
     false,
     2,
     6,
     [](FdpOperator::DomainVector& children,
        FeasibleDomain* self,
        const OperatorSample& /*sample*/) {
         return std::make_unique<FdpRemOperator>(test::TestOperatorStats(), children, self);
     }},
    {"bv_shl",
     node::Kind::BV_SHL,
     2,
     false,
     false,
     2,
     6,
     [](FdpOperator::DomainVector& children,
        FeasibleDomain* self,
        const OperatorSample& /*sample*/) {
         return std::make_unique<FdpLeftShiftOperator>(test::TestOperatorStats(), children, self);
     }},
    {"bv_shr",
     node::Kind::BV_SHR,
     2,
     false,
     false,
     2,
     6,
     [](FdpOperator::DomainVector& children,
        FeasibleDomain* self,
        const OperatorSample& /*sample*/) {
         return std::make_unique<FdpRightShiftOperator>(test::TestOperatorStats(), children, self);
     }},
    {"bv_ashr",
     node::Kind::BV_ASHR,
     2,
     false,
     false,
     2,
     6,
     [](FdpOperator::DomainVector& children,
        FeasibleDomain* self,
        const OperatorSample& /*sample*/) {
         return std::make_unique<FdpArithRightShiftOperator>(test::TestOperatorStats(), children, self);
     }},
    {"bv_and",
     node::Kind::BV_AND,
     2,
     false,
     false,
     1,
     6,
     [](FdpOperator::DomainVector& children,
        FeasibleDomain* self,
        const OperatorSample& /*sample*/) {
         return std::make_unique<FdpAndOperator>(test::TestOperatorStats(), children, self);
     }},
    {"bv_xor",
     node::Kind::BV_XOR,
     2,
     false,
     false,
     1,
     6,
     [](FdpOperator::DomainVector& children,
        FeasibleDomain* self,
        const OperatorSample& /*sample*/) {
         return std::make_unique<FdpXorOperator>(test::TestOperatorStats(), children, self);
     }},
    {"bool_and",
     node::Kind::AND,
     2,
     true,
     true,
     1,
     1,
     [](FdpOperator::DomainVector& children,
        FeasibleDomain* self,
        const OperatorSample& /*sample*/) {
         return std::make_unique<FdpAndOperator>(test::TestOperatorStats(), children, self);
     }},
    {"bool_xor",
     node::Kind::XOR,
     2,
     true,
     true,
     1,
     1,
     [](FdpOperator::DomainVector& children,
        FeasibleDomain* self,
        const OperatorSample& /*sample*/) {
         return std::make_unique<FdpXorOperator>(test::TestOperatorStats(), children, self);
     }},
    {"bool_not",
     node::Kind::NOT,
     1,
     true,
     true,
     1,
     1,
     [](FdpOperator::DomainVector& children,
        FeasibleDomain* self,
        const OperatorSample& /*sample*/) {
         return std::make_unique<FdpNotOperator>(test::TestOperatorStats(), children, self);
     }},
    {"bool_eq",
     node::Kind::EQUAL,
     2,
     true,
     true,
     1,
     1,
     [](FdpOperator::DomainVector& children,
        FeasibleDomain* self,
        const OperatorSample& /*sample*/) {
         return std::make_unique<FdpEqOperator>(test::TestOperatorStats(), children, self);
     }},
    {"bv_ult",
     node::Kind::BV_ULT,
     2,
     false,
     true,
     2,
     6,
     [](FdpOperator::DomainVector& children,
        FeasibleDomain* self,
        const OperatorSample& /*sample*/) {
         return std::make_unique<FdpLessThanOperator>(test::TestOperatorStats(), children, self);
     }},
    {"bv_slt",
     node::Kind::BV_SLT,
     2,
     false,
     true,
     2,
     6,
     [](FdpOperator::DomainVector& children,
        FeasibleDomain* self,
        const OperatorSample& /*sample*/) {
         return std::make_unique<FdpSignedLessThanOperator>(test::TestOperatorStats(), children, self);
     }},
    {"eq",
     node::Kind::EQUAL,
     2,
     false,
     true,
     1,
     6,
     [](FdpOperator::DomainVector& children,
        FeasibleDomain* self,
        const OperatorSample& /*sample*/) {
         return std::make_unique<FdpEqOperator>(test::TestOperatorStats(), children, self);
     }},
    {"bv_concat",
     node::Kind::BV_CONCAT,
     2,
     false,
     false,
     1,
     4,
     [](FdpOperator::DomainVector& children,
        FeasibleDomain* self,
        const OperatorSample& /*sample*/) {
         return std::make_unique<FdpConcatOperator>(test::TestOperatorStats(), children, self);
     },
     MakeConcatSample},
    {"bv_extract",
     node::Kind::BV_EXTRACT,
     1,
     false,
     false,
     3,
     6,
     [](FdpOperator::DomainVector& children,
        FeasibleDomain* self,
        const OperatorSample& sample) {
         const uint64_t high = sample.indices[0];
         const uint64_t low = sample.indices[1];
         return std::make_unique<FdpExtractOperator>(test::TestOperatorStats(), children, self, low, high);
     },
     MakeExtractSample,
     [](NodeManager& nm, const OperatorSample& sample, const std::vector<Node>& terms) {
         return nm.mk_node(node::Kind::BV_EXTRACT, {terms[0]}, sample.indices);
     }},
    {"ite_bv",
     node::Kind::ITE,
     3,
     false,
     false,
     2,
     6,
     [](FdpOperator::DomainVector& children,
        FeasibleDomain* self,
        const OperatorSample& /*sample*/) {
         return std::make_unique<FdpIteOperator>(test::TestOperatorStats(), children, self);
     },
     MakeIteSample},
};

INSTANTIATE_TEST_SUITE_P(FdpOperatorSoundness,
                         FdpOperatorSoundnessTest,
                         ::testing::ValuesIn(kOperatorSpecs),
                         [](const ::testing::TestParamInfo<OperatorSpec>& info) {
                             return info.param.name;
                         });

}  // namespace
}  // namespace bzla::preprocess::pass::fdp

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
