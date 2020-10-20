     
def parse_float(a):
    """ Converts to a float value; raise TypeError. """
    try:
        res = float(a)
        return res
    except:
        msg = 'Cannot convert value to a float.\nValue: %s' % a.__repr__()
        raise TypeError(msg) 
    