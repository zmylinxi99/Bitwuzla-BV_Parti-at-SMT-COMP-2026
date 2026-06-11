
#include <cstdint>
#include <string>
#include <vector>

#include "bv/bitvector.h"
#include "fdp_test_utils.h"
#include "option/option.h"
#include "preprocess/pass/feasible_domain_propagator/fdp_utility.h"
#include "preprocess/pass/feasible_domain_propagator/feasible_domain.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_arithmetic.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_base.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_boolean.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_comparison.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_composition.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_division.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_ite.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_multiplication.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_shifting.h"
#include "printer/printer.h"
#include "solving_context.h"

namespace bzla::preprocess::pass::fdp {
namespace {
using ::bzla::BitVector;
using ::bzla::Node;
using ::bzla::Type;

using DomainVector = std::vector<FeasibleDomain *>;

std::string infos_str = R"(
  before child[0]: width=5 fixed=1/5 bits=[???1?] interval=[2,1f] last_interval=1f pending=unchanged full=0
  before child[1]: width=5 fixed=2/5 bits=[0??0?] interval=[0,d] last_interval=d pending=unchanged full=0
  before output: width=5 fixed=1/5 bits=[???0?] interval=[0,1d] last_interval=1f pending=unchanged full=0
)";

std::string get_substring_between(const std::string &str,
                                  const std::string &start_delim,
                                  const std::string &end_delim) {
    size_t start_pos = str.find(start_delim);
    if (start_pos == std::string::npos) {
        return "";
    }
    start_pos += start_delim.length();

    size_t end_pos = str.find(end_delim, start_pos);
    if (end_pos == std::string::npos) {
        return "";
    }

    return str.substr(start_pos, end_pos - start_pos);
}

FeasibleDomain create_from_string(std::string info) {
    std::string width_str = get_substring_between(info, "width=", " fixed=");
    uint32_t width = std::stoul(width_str);

    FeasibleDomain fd(width);

    std::string bits_str = get_substring_between(info, "bits=[", "] interval=");

    bool comp = info.find("interval=~") != std::string::npos;
    std::string interval_start_delim = comp ? "interval=~[" : "interval=[";

    std::string min_str =
        get_substring_between(info, interval_start_delim, ",");
    std::string max_str =
        get_substring_between(info, ",", "] last_interval=");

    for (uint32_t i = 0; i < width; ++i) {
        char c = bits_str[width - i - 1];
        if (c == '0')
            fd.set_fixed(i, false);

        else if (c == '1')
            fd.set_fixed(i, true);
    }

    BitVector min = BitVector(width, min_str, 16);
    BitVector max = BitVector(width, max_str, 16);

    fd.apply_interval_constraint(min, max, comp);

    return fd;
}

std::vector<std::string> read_info() {
    std::vector<std::string> lines;
    std::stringstream ss(infos_str);
    std::string line;

    // Split the multi-line string into individual lines
    while (std::getline(ss, line)) {
        // Trim leading/trailing whitespace
        size_t first = line.find_first_not_of(" \t\n\r");
        if (std::string::npos == first) {
            continue;
        }
        size_t last = line.find_last_not_of(" \t\n\r");
        lines.push_back(line.substr(first, (last - first + 1)));
    }

    return lines;
}

void manual_test() {
    std::vector<std::string> lines = read_info();

    DomainVector inputs;

    for (uint32_t i = 0; i < lines.size() - 1; ++i) {
        FeasibleDomain fd = create_from_string(lines[i]);
        inputs.push_back(new FeasibleDomain(fd));
    }
    FeasibleDomain output = create_from_string(lines.back());

    for (uint32_t i = 0; i < inputs.size(); ++i) {
        std::cout << "Input Domain " << i << ": " << inputs[i]->to_string()
                  << std::endl;
    }
    std::cout << "Output Domain: " << output.to_string() << std::endl;

    FdpArithRightShiftOperator ashr(test::TestOperatorStats(), inputs, &output);
    ashr.apply();

    for (uint32_t i = 0; i < inputs.size(); ++i) {
        std::cout << "Input Domain " << i << ": " << inputs[i]->to_string()
                  << std::endl;
    }
    std::cout << "Output Domain: " << output.to_string() << std::endl;
}

}  // namespace
}  // namespace bzla::preprocess::pass::fdp

int main() {
    bzla::preprocess::pass::fdp::manual_test();
    return 0;
}