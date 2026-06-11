# Bitwuzla-BV_Parti at SMT-COMP 2026

We intend to participate in SMT-COMP 2026 by submitting **Bitwuzla-BV_Parti** for the **QF_BV Parallel Track** category.

- Solver type: **wrapper solver**
- Base solver: **Bitwuzla 0.9.1**
- Authors: **Mengyu Zhao, Zhenghang Xu, and Shaowei Cai**
- System description: `Bitwuzla-BV_Parti.pdf`
- Pseudo-random 32-bit unsigned number: **998244353**
- Zenodo DOI: 10.5281/zenodo.20636805

## BV_Parti: Feasible-Domain Guided Partitioning for QF_BV

**BV_Parti** is a wrapper solver that adds a lightweight divide-and-conquer layer around Bitwuzla. It keeps Bitwuzla as the base solver for generated SMT-LIB subtasks, while a master process manages task generation, scheduling, timeouts, and result aggregation.

BV_Parti follows our earlier work on dynamic variable-level partitioning, introduced in the CAV 2024 paper *Distributed SMT Solving Based on Dynamic Variable-level Partitioning*. That work studied arithmetic theories. In BV_Parti, we explore how the same partitioning idea can be adapted to fixed-size bit-vectors.

For QF_BV formulas, BV_Parti uses lightweight feasible-domain information over bit-vector terms, such as fixed bits and modular intervals, to select useful partition guards. The generated child tasks are simplified and solved by Bitwuzla.

The detailed description of the current QF_BV method has not yet been published; a full technical paper will be prepared separately.

## How to Build and Test

Download the solver binary files from the submitted artifact package or Zenodo record.

Install MPI-related Python dependencies:

```bash
apt-get install python3-mpi4py
```

Example command:

```bash
./solver/run_BV_Parti.py 128 test/instances/bv-unsat-3.05.smt2
```
