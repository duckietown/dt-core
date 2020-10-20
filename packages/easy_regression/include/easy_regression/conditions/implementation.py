import duckietown_utils as dtu
from easy_regression.conditions.binary import parse_binary
from easy_regression.conditions.eval import BinaryEval, Wrapper
from easy_regression.conditions.interface import RTParseError
from easy_regression.conditions.references import parse_reference


def _parse_regression_test_check(line):
    line = line.strip()
    delim = ' '
    tokens = line.split(delim)

    if len(tokens) != 3:
        msg = 'I expect exactly 3 tokens with delimiter %s.\nLine: "%s"\nTokens: %s' % (delim, line, tokens)
        raise dtu.DTConfigException(msg)

    try:
        ref1 = parse_reference(tokens[0])
        binary = parse_binary(tokens[1])
        ref2 = parse_reference(tokens[2])
        evaluable = BinaryEval(ref1, binary, ref2)
    except RTParseError as e:
        msg = 'Cannot parse string "%s".' % line
        dtu.raise_wrapped(RTParseError, e, msg, compact=True)
    return Wrapper(evaluable)
