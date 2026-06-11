import os
from mpi4py import MPI
from leader import Leader
from coordinator import Coordinator

if __name__ == '__main__':
    os.sched_setaffinity(0, set(range(os.cpu_count())))
    rank = MPI.COMM_WORLD.Get_rank()
    leader_rank = MPI.COMM_WORLD.Get_size() - 1
    # print(f'rank: {rank}')
    if rank < leader_rank:
        ap_coordinator = Coordinator()
        ap_coordinator()
    elif rank == leader_rank:
        ap_leader = Leader()
        ap_leader()
    else:
        assert(False)
    MPI.Finalize()