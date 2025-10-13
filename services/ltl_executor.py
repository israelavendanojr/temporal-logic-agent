import re
import logging
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


logger = logging.getLogger(__name__)


@dataclass
class Action:
    type: str
    target: Optional[str] = None
    duration: Optional[float] = None
    params: Dict[str, Any] = field(default_factory=dict)


@dataclass
class Constraint:
    type: str
    predicate: str
    params: Dict[str, Any]


@dataclass
class ExecutionPlan:
    actions: List[Action] = field(default_factory=list)
    constraints: List[Constraint] = field(default_factory=list)
    temporal_structure: str = "sequential"


class LTLExecutor:
    """Lightweight LTL executor that parses a subset of LTL into an ExecutionPlan.

    This is intentionally simple and regex-driven to support the PoC pipeline.
    """

    # Precompiled regex patterns for supported constructs
    _PATTERN_F_AT = re.compile(r"F\(\s*at\((?P<wp>\w+)\)\s*\)")
    _PATTERN_G_ABOVE = re.compile(r"G\(\s*above\((?P<alt>[0-9.]+)\)\s*\)")
    _PATTERN_G_BELOW = re.compile(r"G\(\s*below\((?P<alt>[0-9.]+)\)\s*\)")
    _PATTERN_F_HOVER = re.compile(r"F\(\s*hover\((?P<dur>[0-9.]+)\)\s*\)")
    _PATTERN_F_SCAN = re.compile(r"F\(\s*scan\((?P<area>\w+)\)\s*\)")
    _PATTERN_F_ACTION = re.compile(r"F\(\s*(?P<action>move_to|land|emergency_return)\((?P<arg>\w+)?\)\s*\)")
    _PATTERN_UNTIL_FAT_FAT = re.compile(r"F\(\s*at\((?P<a>\w+)\)\s*\)\s*U\s*F\(\s*at\((?P<b>\w+)\)\s*\)")
    _PATTERN_AND = re.compile(r"(?P<left>.+?)\s*&\s*(?P<right>.+)")
    _PATTERN_F_COMPLEX_SCAN_NEXT = re.compile(r"F\(\s*at\((?P<wp>\w+)\)\s*&\s*X\(\s*scan\((?P<area>\w+)\)\s*\)\s*\)")
    _PATTERN_G_CLEAR_OF = re.compile(r"G\(\s*clear_of\((?P<obs>\w+)\)\s*\)")

    def parse_formula(self, ltl_formula: str) -> ExecutionPlan:
        """Parse an LTL string into an ExecutionPlan using simple pattern matching.

        The strategy is greedy and handles a useful subset:
        - F(at(X))                -> move_to(X)
        - G(above(Z))             -> altitude constraint (min)
        - G(below(Z))             -> altitude constraint (max)
        - F(hover(t))             -> hover(t)
        - F(scan(A))              -> scan(A)
        - F(at(A)) U F(at(B))     -> [move_to(A), move_to(B)]
        - F(at(A) & X(scan(B)))   -> [move_to(A), scan(B)]
        - F(at(A)) & G(above(Z))  -> action with constraint
        - G(clear_of(obs))        -> obstacle constraint
        Unknown fragments are ignored with a warning to keep PoC resilient.
        """

        if not ltl_formula:
            return ExecutionPlan()

        formula = ltl_formula.strip()

        # First, try the dedicated complex patterns
        complex_match = self._PATTERN_F_COMPLEX_SCAN_NEXT.fullmatch(formula)
        if complex_match:
            wp = complex_match.group("wp")
            area = complex_match.group("area")
            return ExecutionPlan(
                actions=[Action(type="move_to", target=wp), Action(type="scan", target=area)],
                temporal_structure="sequential",
            )

        # Handle binary UNTIL for two waypoints
        until_match = self._PATTERN_UNTIL_FAT_FAT.fullmatch(formula)
        if until_match:
            a = until_match.group("a")
            b = until_match.group("b")
            return ExecutionPlan(
                actions=[Action(type="move_to", target=a), Action(type="move_to", target=b)],
                temporal_structure="sequential",
            )

        # Handle conjunctions by splitting once and recursively parsing parts,
        # then combining actions and constraints
        and_match = self._PATTERN_AND.fullmatch(formula)
        if and_match:
            left = and_match.group("left").strip()
            right = and_match.group("right").strip()
            left_plan = self.parse_formula(left)
            right_plan = self.parse_formula(right)
            return ExecutionPlan(
                actions=left_plan.actions + right_plan.actions,
                constraints=left_plan.constraints + right_plan.constraints,
                temporal_structure="sequential",
            )

        # Single-clause handlers
        actions: List[Action] = []
        constraints: List[Constraint] = []

        # F(at(X))
        m = self._PATTERN_F_AT.fullmatch(formula)
        if m:
            actions.append(Action(type="move_to", target=m.group("wp")))
            return ExecutionPlan(actions=actions, constraints=constraints, temporal_structure="sequential")

        # G(above(Z))
        m = self._PATTERN_G_ABOVE.fullmatch(formula)
        if m:
            try:
                alt = float(m.group("alt"))
            except ValueError:
                alt = None
            if alt is not None:
                constraints.append(Constraint(type="altitude", predicate="above", params={"min": alt}))
            return ExecutionPlan(actions=actions, constraints=constraints, temporal_structure="sequential")

        # G(below(Z))
        m = self._PATTERN_G_BELOW.fullmatch(formula)
        if m:
            try:
                alt = float(m.group("alt"))
            except ValueError:
                alt = None
            if alt is not None:
                constraints.append(Constraint(type="altitude", predicate="below", params={"max": alt}))
            return ExecutionPlan(actions=actions, constraints=constraints, temporal_structure="sequential")

        # G(clear_of(obstacle))
        m = self._PATTERN_G_CLEAR_OF.fullmatch(formula)
        if m:
            constraints.append(Constraint(type="obstacle", predicate="clear_of", params={"object": m.group("obs")}))
            return ExecutionPlan(actions=actions, constraints=constraints, temporal_structure="sequential")

        # F(hover(t))
        m = self._PATTERN_F_HOVER.fullmatch(formula)
        if m:
            try:
                dur = float(m.group("dur"))
            except ValueError:
                dur = 0.0
            actions.append(Action(type="hover", duration=dur))
            return ExecutionPlan(actions=actions, constraints=constraints, temporal_structure="sequential")

        # F(scan(area))
        m = self._PATTERN_F_SCAN.fullmatch(formula)
        if m:
            actions.append(Action(type="scan", target=m.group("area")))
            return ExecutionPlan(actions=actions, constraints=constraints, temporal_structure="sequential")

        # F(move_to(X)) | F(land()) | F(emergency_return())
        m = self._PATTERN_F_ACTION.fullmatch(formula)
        if m:
            action_name = m.group("action")
            arg = m.group("arg")
            if action_name == "move_to":
                actions.append(Action(type="move_to", target=arg))
            elif action_name == "land":
                actions.append(Action(type="land"))
            elif action_name == "emergency_return":
                actions.append(Action(type="emergency_return"))
            return ExecutionPlan(actions=actions, constraints=constraints, temporal_structure="sequential")

        # Fallback: attempt to extract multiple clauses by splitting on '&' that
        # may not have been captured by the fullmatch above (e.g., nested spaces)
        if "&" in formula:
            parts = [p.strip() for p in formula.split("&") if p.strip()]
            combined = ExecutionPlan()
            for part in parts:
                subplan = self.parse_formula(part)
                combined.actions.extend(subplan.actions)
                combined.constraints.extend(subplan.constraints)
            return combined

        logger.warning("Unsupported or unrecognized LTL pattern: %s", formula)
        return ExecutionPlan()


