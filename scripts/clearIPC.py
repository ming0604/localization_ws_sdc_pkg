import sysv_ipc as ipc

shm = ipc.SharedMemory(666, ipc.IPC_CREAT)
shm.remove()
print('shm is cleared.')

sem = ipc.Semaphore(777, ipc.IPC_CREAT)
sem.remove()
print('sem is cleared.')

shmros = ipc.SharedMemory(888, ipc.IPC_CREAT)
shmros.remove()
print('shm_ros is cleared.')

semros = ipc.Semaphore(999, ipc.IPC_CREAT)
semros.remove()
print('sem_ros is cleared.')

semrendezvous = ipc.Semaphore(555, ipc.IPC_CREAT)
semrendezvous.remove()
print('sem_rendezvous is cleared.')
