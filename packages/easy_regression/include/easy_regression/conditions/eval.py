from abc import ABCMeta, abstractmethod
from contextlib import contextmanager

import duckietown_utils as dtu
from .interface import RTCheck, CheckResult
from .result_db import ResultDB, AmbiguousQuery 


class EvaluationError(Exception):
    pass

class DataNotFound(Exception):
    pass


class Evaluable(object):
    __metaclass__ = ABCMeta
    
    @abstractmethod
    @dtu.contract(rdb=ResultDB)
    def eval(self, rdb):
        """ Raise EvaluationError or DataNotFound """

class ResultWithDescription():
    def __init__(self, value, desc):
        self.value = value
        self.desc = desc
    
    def __bool__(self):
        return self.value
        
    def __str__(self):
        return '%s { %s }' % (self.value, self.desc)
        

class Wrapper(RTCheck):
    
    def __init__(self, evaluable):
        self.evaluable = evaluable
        
    def __str__(self):
        return self.evaluable.__str__()
    
    # @contract(returns=CheckResult, result_db=ResultDB)
    def check(self, rdb):
        """ 
            Returns a CheckResult, or raises
            RegressionTestCheckException
            if an abnormal situation is encountered.
        """
        
        try: 
            res = self.evaluable.eval(rdb)
            if not isinstance(res, ResultWithDescription):
                msg = 'Expected ResultWithDescription, obtained %s' % res.__repr__()
                return CheckResult(
                   status=RTCheck.ABNORMAL,
                   summary='Invalid test',
                   details=msg) 
            if res.__bool__() == True:
                return CheckResult(
                   status=RTCheck.OK,
                   summary='OK',
                   details=res.desc)
            if res.__bool__() == False:
                return CheckResult(
                   status=RTCheck.FAIL,
                   summary='Failed',
                   details=res.desc)
        except AmbiguousQuery as e:
            return CheckResult(
               status=RTCheck.FAIL,
               summary='Ambiguous query',
               details=str(e))
        except DataNotFound as e:
            return CheckResult(
               status=RTCheck.NODATA,
               summary='No data available',
               details=str(e))
        except EvaluationError as e:
            return CheckResult(
               status=RTCheck.ABNORMAL,
               summary='Invalid test',
               details=str(e))
        
    

class BinaryEval(Evaluable):
    
    @dtu.contract(a=Evaluable, b=Evaluable)
    def __init__(self, a, op, b):
        self.a = a
        self.op = op
        self.b = b
        
    def eval(self, test_results):
        
        @contextmanager
        def r(m):
            try:
                yield
            except EvaluationError as e:
                msg = 'Cannot evaluate binary operation: error during %s' % m
                msg += '\n' + str(self)
                dtu.raise_wrapped(EvaluationError, e, msg, compact=True)
                
        with r('first operator evaluation'):
            a = self.a.eval(test_results)
        
        with r('second operator evaluation'):
            b = self.b.eval(test_results)
        
        with r('binary evaluation'):
            return self.op(a,b)
    
    def __str__(self):
        s = "Binary operation"
        s += '\n' + dtu.indent(self.a, '', '   a: ')
        s += '\n' + dtu.indent(self.op, '', '  op: ')
        s += '\n' + dtu.indent(self.b, '', '   b: ')
        return s
    
    