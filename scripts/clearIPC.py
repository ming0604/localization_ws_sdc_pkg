import sysv_ipc as ipc

# shm = ipc.SharedMemory(666, ipc.IPC_CREAT)
# shm.remove()
# print('shm is cleared.')

# sem = ipc.Semaphore(777, ipc.IPC_CREAT)
# sem.remove()
# print('sem is cleared.')

# shmros = ipc.SharedMemory(888, ipc.IPC_CREAT)
# shmros.remove()
# print('shm_ros is cleared.')

# semros = ipc.Semaphore(999, ipc.IPC_CREAT)
# semros.remove()
# print('sem_ros is cleared.')

# semrendezvous = ipc.Semaphore(555, ipc.IPC_CREAT)
# semrendezvous.remove()
# print('sem_rendezvous is cleared.')

# ---------------- shm 666 ----------------
try:
    shm = ipc.SharedMemory(666)  
    shm.remove()
    print('shm (666) is cleared.')
except ipc.ExistentialError:
    print('shm (666) does not exist, skipping.')

# ---------------- sem 777 ----------------
try:
    sem = ipc.Semaphore(777)
    sem.remove()
    print('sem (777) is cleared.')
except ipc.ExistentialError:
    print('sem (777) does not exist, skipping.')

# ---------------- shm_ros 888 ----------------
try:
    shmros = ipc.SharedMemory(888)
    shmros.remove()
    print('shm_ros (888) is cleared.')
except ipc.ExistentialError:
    print('shm_ros (888) does not exist, skipping.')

# ---------------- sem_ros 999 ----------------
try:
    semros = ipc.Semaphore(999)
    semros.remove()
    print('sem_ros (999) is cleared.')
except ipc.ExistentialError:
    print('sem_ros (999) does not exist, skipping.')

# ---------------- sem_rendezvous 555 ----------------
try:
    semrendezvous = ipc.Semaphore(555)
    semrendezvous.remove()
    print('sem_rendezvous (555) is cleared.')
except ipc.ExistentialError:
    print('sem_rendezvous (555) does not exist, skipping.')