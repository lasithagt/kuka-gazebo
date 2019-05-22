
import numpy as np
import warnings

# Lifted matrices for MPC on horizon N
def createMPCmatrices(A,B,N,include_x0=1):
    
    # get the size of the A matrix
    n = np.size(A,1)

    # initialize the Ab matrix
    Ab = np.zeros(((N+1)*n, n))
    Ab[0:n,:] = np.eye(n,n)

    for i in range(2,N+2): 
        if (np.size(A,0) > 1000):
            warnings.simplefilter("error")
        Ab[(i-1)*n:i*n,:] = np.matmul(Ab[(i-2)*n:(i-1)*n,:], A)

    if(include_x0 == 0):
        Ab = Ab[n+1:-1,:] # Seems to take a lot of time if A is huge, possible to speedup
    
    m = np.size(B,1)
    Bb = np.zeros(((N+1)*n, N*m))
    print()

    [Bb[0:n,0]] = B

    for i in range(2, N+2):
        print(Bb[(i-2)*n:(i-1)*n,:])
        Bb[(i-1)*n:i*n,:] = np.matmul(A, Bb[(i-2)*n:(i-1)*n,:])
        Bb[(i-1)*n:n*i,(i-2)*m+1:m*(i-1)] = B
    
    if (include_x0 == 0):
        Bb = Bb[n+1:-1,:]

    return Ab, Bb

# For testing
if __name__ == '__main__':

    A = np.array([[1,0],[5,1]])
    B = np.array([[3],[1]])

    (Ab, Bb) = createMPCmatrices(A,B,2)
    print(Ab)
    print(Bb)

    


