

class ExecutionPlan:

    def __init__(self, cnum, mnum):
        self.carts = [list() for _ in range(cnum)]
        self.manips = [list() for _ in range(mnum)]

    def __str__(self):
        res = list()
        res.append('carts:')
        for cart in self.carts:
            res.append(str(cart))
        res.append('manips:')
        for manip in self.manips:
            res.append(str(manip))
        return '\n'.join(res)
