import numpy as np
from typing import List

class FuzzyPID:
    '''
    PID controller class
    '''
    def __init__(self):
        self.error_bef = 0
        self.p : List[float] = [0.0, 0,2]
        self.i : List[float] = [0.0, 0.0]
        self.d : List[float] = [0.0, 0.0]
        self.error_accum = 0
        self.lower_bound = -np.pi/2
        self.upper_bound = np.pi/2
        self.accum_lower_bound = -1.0
        self.accum_upper_bound = 1.0
        self.clamp = lambda x : max(min(x,self.upper_bound),self.lower_bound)
        self.clamp_accum = lambda x : max(min(x,self.accum_upper_bound),self.accum_lower_bound)

    def set_pid(self, kp: List[float], ki: List[float], kd: List[float]):
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
        if err < np.pi/8 and err > -np.pi/8:
            ret = self.p[0]*err + self.i[0]*self.error_accum + self.d[0]*(self.error_bef-err) 
        else:
            ret = self.p[1]*err + self.i[1]*self.error_accum + self.d[1]*(self.error_bef-err)
        ret = self.clamp(ret)
        self.error_bef = ret
        return ret