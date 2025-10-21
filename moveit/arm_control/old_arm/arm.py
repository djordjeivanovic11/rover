import numpy as np
from numpy.linalg import matrix_power
from scipy.linalg import expm, norm, inv, pinv
import scipy.linalg
from scipy.spatial.transform import Rotation
import warnings
warnings.filterwarnings('ignore')

np.set_printoptions(suppress=True)

MAX_ITERS = 100

def logm(M): #self defined SE(3) matrix log (numpy version does not work the same way)
    #ignoring identity case because it is impossible in the Newton-Raphson
    R = M[0:3,0:3]
    p = M[0:3,3]
    w = np.array([0,0,0])
    sk_w = np.zeros((3,3))
    th = 0
    if np.array_equal(R,np.identity(3)):
        pass #should never happen as 
    elif np.trace(R) == -1:
        th = np.pi
        choice = np.where(np.array([R[0,0],R[1,1],R[2,2]]) != -1)[0][0]
        w = np.zeros(3)
        w[choice] = 1
        w += np.array([R[0,choice],R[1,choice],R[2,choice]])
        w/=np.sqrt(2*(1+R[choice,choice]))
        sk_w = np.array([[0,-w[2],w[1]],
                     [w[2],0,-w[0]],
                     [-w[1],w[0],0]])
    else:
        th = np.arccos(complex((np.trace(R)-1)/2))
        sk_w = (R-np.transpose(R))/(2*np.sin(th))

    G = np.identity(3)/th - sk_w/2 + (1/th - 1/(2*np.tan(th/2)))*matrix_power(sk_w,2)
    v = G.dot(p)
    S = np.zeros((4,4))
    S[0:3,0:3] = sk_w*th
    S[0:3,3] = v*th
    return S

def adjoint(M):
    R = M[0:3,0:3]
    adj = np.zeros((6,6))
    adj[0:3,0:3] = R
    p = np.array([[0,-M[2,3],M[1,3]],
                  [M[2,3],0,-M[0,3]],
                  [-M[1,3],M[0,3],0]])
    adj[3:,0:3] = p @ R
    adj[3:,3:] = R
    return adj

class Joint():
    def __init__(self,w,q):
        self.w = np.array(w)
        self.q = np.array(q)
        self.v = np.cross(-self.w,self.q)
        self.S = np.array([[0,-self.w[2],self.w[1],self.v[0]], #matrix representation of screw axis: [w v]
                              [self.w[2],0,-self.w[0],self.v[1]],
                              [-self.w[1],self.w[0],0,self.v[2]],
                              [0, 0, 0, 0]])
        self.Svec = np.zeros(6)
        self.Svec[:3] = self.w
        self.Svec[3:] = self.v
    def T(self,th): #transformation given rotation of joint by angle "th"
        return expm(self.S*th)

class Arm():
    def __init__(self,M,*joints):
        self.joints = joints
        self.M = M
        self.adM = adjoint(inv(M))
        self.currentTheta = [0 for i in range(len(self.joints))]
        self.currentTheta[0] = np.pi/2

        #generate body twists from space twists of joints
        self.B = []
        for j in self.joints:
            self.B.append(self.adM.dot(j.Svec))
    def FK(self,theta): #generic forward kinematics, given angles of each joint, compute end effector twist
        assert(len(theta)==len(self.joints))
        POE = self.M
        for th,joint in zip(theta[::-1],self.joints[::-1]):
            POE = joint.T(th) @ POE
        return POE
    def setAngles(self,theta):
        self.currentTheta = theta
    def J(self,th):
        tot = np.identity(4)
        Jout = np.zeros((6,len(th)))
        i = len(th)-1
        Jout[:,i] = self.B[i]
        for angle,b in zip(th[1:][::-1],self.B[1:][::-1]):
            Bmat = np.array([[0,-b[2],b[1],b[3]], #matrix representation of body screw axis
                              [b[2],0,-b[0],b[4]],
                              [-b[1],b[0],0,b[5]],
                              [0, 0, 0, 0]])
            tot = tot @ expm(-angle*Bmat)
            i-=1
            Jout[:,i] = adjoint(tot).dot(self.B[i])
        return Jout

    def IK(self,pos,rotX,rotY,rotZ,ep=0.001):
        T = np.zeros((4,4))
        T[0:3,0:3] = Rotation.from_euler('xyz',[rotX,rotY,rotZ],degrees=False).as_matrix()
        T[3,3] = 1
        T[0:3,3] = pos
        guess = np.array(self.currentTheta.copy()).astype(float)
        bTwist = scipy.linalg.logm(inv(self.FK(guess)) @ T)
        vbTwist = np.array([bTwist[2,1],bTwist[0,2],bTwist[1,0],bTwist[0,3],bTwist[1,3],bTwist[2,3]])
        iters = 0
        while (norm(vbTwist[:3])>ep or norm(vbTwist[3:])>ep) and iters<MAX_ITERS:
            guess += pinv(self.J(guess)).dot(vbTwist)
            bTwist = scipy.linalg.logm(inv(self.FK(guess)) @ T)
            vbTwist = np.array([bTwist[2,1],bTwist[0,2],bTwist[1,0],bTwist[0,3],bTwist[1,3],bTwist[2,3]])
            iters+=1
            if np.any(np.isnan(vbTwist)|np.isinf(vbTwist)):
                return None
        if iters>=MAX_ITERS:
            return None
        guess %= 2*np.pi
        return guess
        
if __name__ == "__main__":
    #initialize screw axes for each joint
    #all numbers taken from UR5e (refer to diagram)
    #change as needed to match the robot's arm
    W2 = 259.6
    W1 = 133.3
    H2 = 99.7
    H1 = 162.5
    L1 = 425
    L2 = 392.2
    
    joint1 = Joint([0,0,1],[0,0,0])
    joint2 = Joint([0,1,0],[0,0,H1])
    joint3 = Joint([0,1,0],[L1,0,H1])
    joint4 = Joint([0,1,0],[L1+L2,0,H1])
    joint5 = Joint([0,0,-1],[L1+L2,W1,0])
    joint6 = Joint([0,1,0],[L1+L2,0,H1-H2])

    #initialize transformation matrix from origin to end effector at zero position for arm
    M = np.array([[-1,0,0,L1+L2],
                [0,0,1,W1+W2],
                [0,1,0,H1-H2],
                [0,0,0,1]])

    #initialize arm with M matrix and every joint
    arm = Arm(M,joint1,joint2,joint3,joint4,joint5,joint6)

    #run inverse kinematics on this position from arm
    ik = arm.IK([720.0903,-350.0874,64.4169],0,np.pi,0)
    print(ik)
    print(arm.FK(ik))
        