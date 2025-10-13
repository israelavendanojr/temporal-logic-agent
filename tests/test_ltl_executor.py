import unittest

from services.ltl_executor import LTLExecutor


class TestLTLExecutor(unittest.TestCase):
    def setUp(self):
        self.exec = LTLExecutor()

    def test_parse_F_at(self):
        plan = self.exec.parse_formula("F(at(waypoint_a))")
        self.assertEqual(len(plan.actions), 1)
        self.assertEqual(plan.actions[0].type, "move_to")
        self.assertEqual(plan.actions[0].target, "waypoint_a")

    def test_parse_G_above(self):
        plan = self.exec.parse_formula("G(above(2.0))")
        self.assertEqual(len(plan.constraints), 1)
        c = plan.constraints[0]
        self.assertEqual(c.type, "altitude")
        self.assertEqual(c.params.get("min"), 2.0)

    def test_until_sequence(self):
        plan = self.exec.parse_formula("F(at(A)) U F(at(B))")
        self.assertEqual([a.target for a in plan.actions], ["A", "B"])

    def test_and_action_and_constraint(self):
        plan = self.exec.parse_formula("F(at(A)) & G(above(1.5))")
        self.assertEqual(len(plan.actions), 1)
        self.assertEqual(plan.actions[0].target, "A")
        self.assertEqual(plan.constraints[0].params.get("min"), 1.5)

    def test_complex_move_then_scan(self):
        plan = self.exec.parse_formula("F(at(A) & X(scan(area_1)))")
        self.assertEqual([a.type for a in plan.actions], ["move_to", "scan"])
        self.assertEqual(plan.actions[0].target, "A")
        self.assertEqual(plan.actions[1].target, "area_1")


if __name__ == "__main__":
    unittest.main()


