import pytest
from agent.ltl.semantic_validator import SemanticValidator

def test_detects_inverted_obstacle():
    validator = SemanticValidator()
    
    # Should fail - inverted negation
    is_valid, error = validator.validate("F(at(waypoint_a)) & G(!clear_of(obstacle_1))")
    assert not is_valid
    assert "Inverted obstacle avoidance" in error
    
    # Should pass - correct form
    is_valid, error = validator.validate("F(at(waypoint_a)) & G(clear_of(obstacle_1))")
    assert is_valid

def test_detects_spatial_conflicts():
    validator = SemanticValidator()
    
    # Should fail - cannot be at two places always
    is_valid, error = validator.validate("G(at(waypoint_a)) & G(at(waypoint_b))")
    assert not is_valid
    
    # Should pass - eventually at two places (sequential)
    is_valid, error = validator.validate("F(at(waypoint_a)) & F(at(waypoint_b))")
    assert is_valid

def test_extracts_waypoints():
    validator = SemanticValidator()
    formula = "F(at(waypoint_a)) & F(at(waypoint_b)) & G(clear_of(obstacle_1))"
    waypoints = validator.extract_waypoints(formula)
    assert waypoints == {'waypoint_a', 'waypoint_b'}

def test_extracts_actions():
    validator = SemanticValidator()
    formula = "F(scan(area_1)) & F(hover(10)) & F(land())"
    actions = validator.extract_actions(formula)
    assert len(actions) == 3
    assert "scan(area_1)" in actions
    assert "hover(10)" in actions
    assert "land()" in actions

def test_perimeter_negation_detection():
    validator = SemanticValidator()
    
    # Should fail - inverted perimeter negation
    is_valid, error = validator.validate("F(at(waypoint_a)) & G(!clear_of(perimeter_A))")
    assert not is_valid
    assert "perimeter_A" in error
    
    # Should pass - correct perimeter avoidance
    is_valid, error = validator.validate("F(at(waypoint_a)) & G(clear_of(perimeter_A))")
    assert is_valid

def test_stationary_moving_conflict():
    validator = SemanticValidator()
    
    # Should fail - conflicting constraints
    is_valid, error = validator.validate("G(stationary()) & F(moving())")
    assert not is_valid
    assert "stationary and moving" in error
    
    # Should pass - no conflict
    is_valid, error = validator.validate("F(stationary()) & F(moving())")
    assert is_valid

def test_redundancy_warning():
    validator = SemanticValidator()
    
    # Should pass but generate warning
    is_valid, warning = validator.validate("F(at(waypoint_a)) & F(at(waypoint_a))")
    assert is_valid  # Redundancy is warning, not error
