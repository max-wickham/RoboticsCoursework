import re


def simplify(string : str) -> str:
    m = re.match('(45)*(36)',"(*)\\*(*)")
    print(m)
    if m:
        print(m.group(1))
    return string

simplify('25')