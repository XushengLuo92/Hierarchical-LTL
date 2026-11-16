"""Independent finite-trace checks for exported planner paths and schedules."""
from functools import lru_cache
from itertools import combinations
import re

class FormulaParser:
    def __init__(self, formula):
        token = r'\[\]|<>|&&|\|\||->|[!()]|[A-Za-z][A-Za-z0-9_]*'
        self.tokens = re.findall(token, formula)
        if ''.join(self.tokens) != re.sub(r'\s+', '', formula):
            raise ValueError('Unsupported LTL token.')
        self.index = 0

    def peek(self):
        return self.tokens[self.index] if self.index < len(self.tokens) else None

    def take(self):
        value = self.peek()
        self.index += 1
        return value

    def parse(self):
        node = self.expression(0)
        if self.peek() is not None:
            raise ValueError(f'Unexpected token {self.peek()}')
        return node

    def expression(self, minimum):
        value = self.take()
        if value in ('!', '[]', '<>', 'X'):
            left = (value, self.expression(5))
        elif value == '(':
            left = self.expression(0)
            if self.take() != ')':
                raise ValueError('Missing closing parenthesis.')
        elif value is not None and re.fullmatch(r'[a-zA-Z][a-zA-Z0-9_]*', value) and value != 'U':
            left = ('atom', value)
        else:
            raise ValueError(f'Unexpected token {value}')
        # Match ltl2ba/parse.c: Boolean binary operators share a level,
        # until binds more tightly, and both levels associate to the left.
        priorities = {'->': 1, '||': 1, '&&': 1, 'U': 2}
        while self.peek() in priorities and priorities[self.peek()] >= minimum:
            operator = self.take()
            priority = priorities[operator]
            right = self.expression(priority + 1)
            left = (operator, left, right)
        return left


def finite_truth(formula, trace):
    """LTLf with strong next/until; no artificial loop or padded final state."""
    if not trace:
        return False
    tree = FormulaParser(formula).parse() if isinstance(formula, str) else formula

    @lru_cache(None)
    def evaluate(node):
        op = node[0]
        if op == 'atom':
            return tuple(node[1] == 'true' or node[1] in row for row in trace)
        a = evaluate(node[1])
        if op == '!':
            return tuple(not value for value in a)
        if op == 'X':
            return a[1:] + (False,)
        if op in ('<>', '[]', 'U'):
            b = evaluate(node[2]) if op == 'U' else None
            value = op == '[]'
            result = []
            for i in reversed(range(len(trace))):
                value = (a[i] and value) if op == '[]' else (a[i] or value) if op == '<>' else (b[i] or (a[i] and value))
                result.append(value)
            return tuple(reversed(result))
        b = evaluate(node[2])
        if op == '&&': return tuple(x and y for x, y in zip(a, b))
        if op == '||': return tuple(x or y for x, y in zip(a, b))
        if op == '->': return tuple(not x or y for x, y in zip(a, b))
        raise ValueError(op)
    return evaluate(tree)[0]


class Monitor:
    def __init__(self, data):
        self.data = data
        self.formulas = {key: value for level in data['hierarchy'] for key, value in level.items()}
        self.children = {key: list(dict.fromkeys(atom.split('_')[0] for atom in
                         re.findall(r'p\d+_\d+_\d+_\d+', value)
                         if int(atom.split('_')[0][1:]) >= 100 and atom.split('_')[0] in self.formulas))
                         for key, value in self.formulas.items()}
        self.leaves = [key for key in self.formulas if not self.children[key]]
        self.stages = data['stages']
        self.horizon = max(s['time_element'] for s in self.stages)
        if data['task'] == 'nav':
            self.horizon = max(len(r['path']) for r in data['robots']) - 1
        self.times = {tuple(s['subtask']): s['time_element'] for s in self.stages}
        self.identities = {i['identifier']: i['robots'] for i in data['identities']}
        self.completed, self.activation, self.selected = {}, {}, set()
        self.errors = []
        for phi in self.leaves:
            stages = [s for s in self.stages if s['subtask'][0] == phi]
            if not stages:
                continue
            self.selected.add(phi)
            self.activation[phi] = max([self.times[tuple(a)] for a, b in data['precedence']
                                       if b[0] == phi and a[0] != phi and tuple(a) in self.times] or [0])
            start = self.activation[phi]
            end = max(s['time_element'] for s in stages)
            atoms = set(re.findall(r'p\d+_\d+_\d+_\d+', self.formulas[phi]))
            trace = [{atom for atom in atoms if self.atom_true(atom, t)} for t in range(start, end+1)]
            if finite_truth(self.formulas[phi], trace):
                # Completion is the first verified prefix in this task's activation window.
                self.completed[phi] = next(start+i for i in range(len(trace))
                                           if finite_truth(self.formulas[phi], trace[:i+1]))
            else:
                self.errors.append({'task': phi, 'reason': 'leaf formula not satisfied', 'window': [start, end]})
        for level in reversed(data['hierarchy']):
            for phi, formula in level.items():
                if not self.children[phi]:
                    continue
                atoms = set(re.findall(r'p\d+_\d+_\d+_\d+', formula))
                trace = [{atom for atom in atoms if self.completed.get(atom.split('_')[0], float('inf')) <= t}
                         for t in range(self.horizon+1)]
                if finite_truth(formula, trace):
                    self.completed[phi] = next(t for t in range(self.horizon+1) if finite_truth(formula, trace[:t+1]))

    def position(self, robot, step):
        return robot['path'][min(step, len(robot['path'])-1)]

    def atom_true(self, atom, step):
        region, kind, count, identity = atom.split('_')
        kind, count, identity = int(kind), int(count), int(identity)
        candidates = [r for r in self.data['robots'] if r['type'] == kind and
                      (identity == 0 or r['index'] in self.identities.get(identity, []))]
        if self.data['task'] == 'nav':
            return sum(self.position(r, step) in self.data['workspace']['regions'][region]
                       for r in candidates) >= count
        assigned = {tuple(r) for s in self.stages if s['time_element'] == step
                    for r in s['essential_robot_edge'].get(region, [])}
        return sum((r['type'], r['index']) in assigned for r in candidates) >= count

    def status(self, phi, step):
        if self.completed.get(phi, float('inf')) <= step:
            return 'done'
        if phi in self.leaves:
            if phi not in self.selected:
                return 'unused'
            return 'active' if self.activation[phi] <= step else 'pending'
        return 'active' if any(self.status(c, step) in ('done', 'active') for c in self.children[phi]) else 'pending'

    def validate(self):
        errors = list(self.errors)
        conflicts = []
        for a, b in self.data['precedence']:
            if tuple(a) in self.times and tuple(b) in self.times and self.times[tuple(a)] >= self.times[tuple(b)]:
                errors.append({'reason': 'precedence', 'before': a, 'after': b})
        if self.data['task'] == 'nav':
            workspace = self.data['workspace']
            blocked = {tuple(p) for cells in workspace['obstacles'].values() for p in cells}
            for r in self.data['robots']:
                for t, p in enumerate(r['path']):
                    if (tuple(p) in blocked or not (0 <= p[0] < workspace['width'] and 0 <= p[1] < workspace['height'])
                        or (t and sum(abs(a-b) for a,b in zip(p,r['path'][t-1])) > 1)):
                        errors.append({'reason': 'illegal move', 'robot': [r['type'],r['index']], 'step': t})
            for t in range(self.horizon+1):
                for a, b in combinations(self.data['robots'], 2):
                    if self.position(a,t) == self.position(b,t) or (t and self.position(a,t) == self.position(b,t-1)
                                                                 and self.position(b,t) == self.position(a,t-1)):
                        conflicts.append({'step': t, 'robots': [[a['type'],a['index']],[b['type'],b['index']]]})
            for stage in self.stages:
                t = stage['time_element']
                for region, robots in stage['essential_robot_edge'].items():
                    for robot in robots:
                        r = next(r for r in self.data['robots'] if [r['type'],r['index']] == robot)
                        if self.position(r,t) not in workspace['regions'][region]:
                            errors.append({'reason': 'missed stage target', 'step': t, 'region': region})
        return {'task': self.data['task'], 'case': self.data['case'], 'horizon': self.horizon,
                'hierarchy_satisfied': 'p0' in self.completed, 'completion_steps': self.completed,
                'activation_steps': self.activation, 'errors': errors, 'collision_conflicts': conflicts,
                'valid': 'p0' in self.completed and not errors and not conflicts,
                'scope': 'grid motion and LTL' if self.data['task'] == 'nav' else 'high-level schedule and LTL; no physical arm validation'}

