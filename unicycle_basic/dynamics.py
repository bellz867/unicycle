import numpy as np
from math import sin
from math import cos

np.random.seed(0)

# class for the two link dynamics
class Dynamics():
    # constructor to initialize a Dynamics object
    def __init__(self):
        """
        Initialize the dynamics \n
        Inputs:
        -------
        \t alpha: error gain \n
        \t beta:  filtered error gain \n
        \t gamma: parameter update gain \n
        \t kCL: CL parameter update gain \n
        
        Returns:
        -------
        """
        # gains
        self.X = np.zeros(4,dtype=np.float32)
        self.X[2] = 1.0
        self.XD = np.zeros(4,dtype=np.float32)
        self.u = np.zeros(2,dtype=np.float32)

        # desired trajectory parameters
        self.magd = 1.0 # radius of circle
        self.fd = 1.0/20.0 # frequency in Hz

        self.maxDiff = 10**(-4)

        #butcher table for ode45 from https://en.wikipedia.org/wiki/Dormand%E2%80%93Prince_method
        #implement from https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods#Explicit_Runge.E2.80.93Kutta_methods
        #c is the time weights, b is the out weights, balt is the alternative out weights, and a is the table weights
        self.BTc = np.array([0,1/5,3/10,4/5,8/9,1,1])
        self.BTb = np.array([35/384,0,500/1113,125/192,-2187/6784,11/84,0])
        self.BTbalt = np.array([5179/57600,0,7571/16695,393/640,-92097/339200,187/2100,1/40])
        self.BTa = np.zeros((7,6),dtype=np.float32)
        self.BTa[1,0] = 1/5
        self.BTa[2,0:2] = [3/40,9/40]
        self.BTa[3,0:3] = [44/45,-56/15,32/9]
        self.BTa[4,0:4] = [19372/6561,-25360/2187,64448/6561,-212/729]
        self.BTa[5,0:5] = [9017/3168,-355/33,46732/5247,49/176,-5103/18656]
        self.BTa[6,0:6] = [35/384,0,500/1113,125/192,-2187/6784,11/84]

    def getDesiredState(self,t):
        """
        Determines the desired state of the system \n
        Inputs:
        -------
        \t t: time \n
        
        Returns:
        -------
        \t phid:   desired angles \n
        \t phiDd:  desired angular velocity \n
        \t phiDDd: desired angular acceleration
        """
        # desired angles
        Xd = np.zeros(4,dtype=np.float32)

        #desired angular velocity
        XDd = np.zeros(4,dtype=np.float32)
        
        return Xd,XDd

    #returns the state
    def getState(self,t):
        """
        Returns the state of the system and parameter estimates \n
        Inputs:
        -------
        \t t: time \n
        
        Returns:
        -------
        \t X:   position and orientation \n
        \t X:   velocities in world \n
        \t u:   velocities in body \n
        """
        return self.X,self.XD,self.u

    #returns the error state
    def getErrorState(self,t):
        """
        Returns the errors \n
        Inputs:
        -------
        \t t:  time \n
        
        Returns:
        -------
        \t e:          tracking error \n
        \t eD:         tracking error derivative \n
        \t r:          filtered tracking error \n
        \t thetaTilde: parameter estimate error
        """
        # get the desired state
        Xd,XDd = self.getDesiredState(t)

        # get the tracking error
        e = Xd - self.X
        eD = XDd - self.XD

        return e,eD

    def getf(self,t,X):
        """
        Dynamics callback \n
        Inputs:
        -------
        \t t:  time \n
        \t X:  stacked x,y,qw,qz \n
        
        Returns:
        -------
        \t XD: derivative approximate at time \n
        \t u: control input at time \n
        """

        x = X[0]
        y = X[1]
        qw = X[2]
        qz = X[3]

        # get the desired state
        # phid,phiDd,phiDDd = self.getDesiredState(t)

        #calculate the controller and update law
        v = 1.0
        w = 2.0*np.pi*1.0/10.0

        XD = np.zeros_like(X)
        XD[0] = (qw*qw-qz*qz)*v
        XD[1] = 2*qw*qz*v
        XD[2] = -0.5*qz*w
        XD[3] = 0.5*qw*w

        return XD

    def getu(self,t,X,XD):
        """
        Dynamics callback \n
        Inputs:
        -------
        \t t:  time \n
        \t X:  stacked x,y,qw,qz \n
        
        Returns:
        -------
        \t XD: derivative approximate at time \n
        \t u: control input at time \n
        """

        x = X[0]
        y = X[1]
        qw = X[2]
        qz = X[3]

        xD = XD[0]
        yD = XD[1]
        qwD = XD[2]
        qzD = XD[3]

        # get the desired state
        # phid,phiDd,phiDDd = self.getDesiredState(t)

        #calculate the controller and update law
        v = (qw*qw-qz*qz)*xD+2*qw*qz*yD
        w = -2*qz*qwD+2*qw*qzD
        u = np.array([v,w])
        return u

    #classic rk1 method aka Euler
    def rk1(self,dt,t,X):
        """
        Classic rk1 method aka Euler \n
        Inputs:
        -------
        \t dt:  total time step for interval \n
        \t t:  time \n
        \t X:  stacked phi,phiD,thetaH \n
        
        Returns:
        -------
        \t XD: derivative approximate over total interval \n
        \t tau: control input approximate over total interval \n
        \t Xh: integrated value \n
        """
        XD = self.getf(t,X)
        Xh = X + dt*XD
        u = self.getu(t,X,XD)

        return XD,Xh,u

    #classic rk4 method
    def rk4(self,dt,t,X):
        """
        Classic rk4 method \n
        Inputs:
        -------
        \t dt:  total time step for interval \n
        \t t:  time \n
        \t X:  stacked phi,phiD,thetaH \n
        
        Returns:
        -------
        \t XD: derivative approximate over total interval \n
        \t tau: control input approximate over total interval \n
        \t Xh: integrated value \n
        """

        k1 = self.getf(t,X)
        k2 = self.getf(t+0.5*dt,X+0.5*dt*k1)
        k3 = self.getf(t+0.5*dt,X+0.5*dt*k2)
        k4 = self.getf(t+dt,X+dt*k3)
        XD = (1.0/6.0)*(k1+2.0*k2+2.0*k3+k4)
        Xh = X+dt*XD

        u = self.getu(t,X,XD)

        return XD,Xh,u

    #adaptive step using classic Dormand Prince method aka rk45 or ode45 method
    def rk45(self,dt,t,X):
        """
        Adaptive step using classic Dormand Prince method aka ode45 method \n
        Inputs:
        -------
        \t dt:  total time step for interval \n
        \t t:  time \n
        \t X:  stacked phi,phiD,thetaH \n
        
        Returns:
        -------
        \t XD: derivative approximate over total interval \n
        \t tau: control input approximate over total interval \n
        \t Xh: integrated value \n
        """

        #initially time step is equal to full dt
        steps = 1
        XDdiff = 100.0
        XD = np.zeros(9,dtype=np.float32)
        Xh = X.copy()
        while XDdiff >= self.maxDiff:
            Xh = X.copy()
            th = t
            h = dt/steps
            for ii in range(steps):
                #calculate the ks and taus
                ks = np.zeros((7,9),np.float32)
                ks[0,:] = self.getf(th,Xh)
                ks[1,:] = self.getf(th+self.BTc[1]*h,Xh+h*(self.BTa[1,0]*ks[0,:]))
                ks[2,:] = self.getf(th+self.BTc[2]*h,Xh+h*(self.BTa[2,0]*ks[0,:]+self.BTa[2,1]*ks[1,:]))
                ks[3,:] = self.getf(th+self.BTc[3]*h,Xh+h*(self.BTa[3,0]*ks[0,:]+self.BTa[3,1]*ks[1,:]+self.BTa[3,2]*ks[2,:]))
                ks[4,:] = self.getf(th+self.BTc[4]*h,Xh+h*(self.BTa[4,0]*ks[0,:]+self.BTa[4,1]*ks[1,:]+self.BTa[4,2]*ks[2,:]+self.BTa[4,3]*ks[3,:]))
                ks[5,:] = self.getf(th+self.BTc[5]*h,Xh+h*(self.BTa[5,0]*ks[0,:]+self.BTa[5,1]*ks[1,:]+self.BTa[5,2]*ks[2,:]+self.BTa[5,3]*ks[3,:]+self.BTa[5,4]*ks[4,:]))
                ks[6,:] = self.getf(th+self.BTc[6]*h,Xh+h*(self.BTa[6,0]*ks[0,:]+self.BTa[6,1]*ks[1,:]+self.BTa[6,2]*ks[2,:]+self.BTa[6,3]*ks[3,:]+self.BTa[6,4]*ks[4,:]+self.BTa[6,5]*ks[5,:]))
                
                #calculate the complete derivate, alternative derivative, and input
                XDh = np.zeros(9,dtype=np.float32)
                XDalth = np.zeros(9,dtype=np.float32)
                for ii in range(7):
                    XDh += self.BTb[ii]*ks[ii,:]
                    XDalth += self.BTbalt[ii]*ks[ii,:]
                            
                th += h
                Xh += h*XDh

                # update the difference 
                XDdiff = np.linalg.norm(XDh-XDalth)
                if XDdiff >= self.maxDiff:
                    print("h ",str(h))
                    print("th ",str(th))
                    print("XD diff ",str(XDdiff))
                    phiDdiff = np.linalg.norm(XDh[0:2]-Xh[2:4])
                    print("phiD diff ",str(phiDdiff))
                    steps += 1
                    break
            if XDdiff < self.maxDiff:
                XD = (1.0/dt)*(Xh-X)

        u = self.getu(t,X,XD)

        return XD,Xh,u

    # take a step of the dynamics
    def step(self,dt,t):
        """
        Steps the internal state using the dynamics \n
        Inputs:
        -------
        \t dt: time step \n
        \t t:  time \n
        
        Returns:
        -------
        """
        
        # update the internal state

        #get the derivative and input from rk
        XD,Xh,u = self.rk1(dt,t,self.X)

        self.X = Xh.copy()
        self.XD = XD.copy()
        self.u = u.copy()
        