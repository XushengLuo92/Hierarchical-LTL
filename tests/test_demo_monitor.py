import copy
import json
from pathlib import Path
import unittest

from hierarchical_ltl.demo_monitor import finite_truth, Monitor

DATA = Path(__file__).resolve().parents[1] / 'demo/generated'


class FormulaTests(unittest.TestCase):
    def test_strong_next_needs_a_real_next_step(self):
        self.assertFalse(finite_truth('a && X b', [{'a', 'b'}]))
        self.assertTrue(finite_truth('a && X b', [{'a'}, {'b'}]))

    def test_until_cannot_skip_a_gap(self):
        self.assertTrue(finite_truth('a U b', [{'a'}, {'a'}, {'b'}]))
        self.assertFalse(finite_truth('a U b', [{'a'}, set(), {'b'}]))

    def test_health_restriction(self):
        formula = '<> health && <> grocery && !grocery U health'
        self.assertTrue(finite_truth(formula, [set(), {'health'}, {'grocery'}]))
        self.assertFalse(finite_truth(formula, [{'grocery'}, {'health'}, {'grocery'}]))


class ReplayTests(unittest.TestCase):
    def load(self, scenario):
        return json.loads((DATA/f'scenario_{scenario:02d}.json').read_text())

    def test_all_original_plans(self):
        for scenario in range(1, 6):
            with self.subTest(scenario=scenario):
                self.assertTrue(Monitor(self.load(scenario)).validate()['valid'])

    def test_wrong_robot_cannot_release_furniture_hold(self):
        data=self.load(1)
        assigned=next(s for s in data['stages'] if s['subtask']==['p201',2])
        robot_id=assigned['essential_robot_edge']['p5'][0]
        assigned_robot=next(r for r in data['robots'] if [r['type'],r['index']]==robot_id)
        other=next(r for r in data['robots'] if r['type']==3 and r['index']!=robot_id[1])
        assigned_robot['path'],other['path']=other['path'],assigned_robot['path']
        monitor=Monitor(data)
        self.assertNotIn('p201',monitor.completed)
        self.assertFalse(monitor.validate()['valid'])

    def test_unselected_or_branch_is_not_failure(self):
        monitor=Monitor(self.load(3))
        self.assertIn('p0',monitor.completed)
        self.assertEqual(sum(monitor.status(phi,monitor.horizon)=='unused' for phi in ('p103','p104')),1)

    def test_assembly_order_violation_is_rejected(self):
        data=self.load(5)
        last=next(s for s in data['stages'] if 'p18' in s['essential_robot_edge'])
        last['time_element']=1
        self.assertFalse(Monitor(data).validate()['valid'])


if __name__=='__main__':
    unittest.main()
