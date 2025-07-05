import numpy as np

class BasicPID:
    '''
    PID controller class
    '''
    def __init__(self):
        self.error_bef = 0
        self.p = 0
        self.i = 0
        self.d = 0
        self.error_accum = 0
        self.lower_bound = -np.pi/2
        self.upper_bound = np.pi/2
        self.accum_lower_bound = -1.0
        self.accum_upper_bound = 1.0
        self.clamp = lambda x : max(min(x,self.upper_bound),self.lower_bound)
        self.clamp_accum = lambda x : max(min(x,self.accum_upper_bound),self.accum_lower_bound)

    def set_pid(self, kp:float, ki:float, kd:float):
        ''''
        param 
            kp: float
            ki: float
            kd: float
        description
            set pid gains
        '''
        self.p = kp
        self.i = ki
        self.d = kd
        

    def reset(self):
        '''
        description
            reset pid controller
        '''
        self.error_bef = 0
        self.error_accum = 0
        
    def update(self,err:float) -> float:
        '''
        param
            err: float
        return
            float
                pid output
        description
            update pid controller
        '''
        self.error_accum += err
        self.error_accum = self.clamp_accum(self.error_accum)
        ret = self.p*err + self.i*self.error_accum + self.d*(self.error_bef-err)
        ret = self.clamp(ret)
        self.error_bef = ret
        return ret