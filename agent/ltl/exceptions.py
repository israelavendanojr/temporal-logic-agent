"""Custom exceptions for LTL processing"""

class LTLError(Exception):
    """Base exception for LTL processing errors"""
    pass

class LTLSyntaxError(LTLError):
    """Raised when LTL syntax is invalid"""
    pass

class LTLSafetyError(LTLError):
    """Raised when safety constraints are violated"""
    pass

class ConfigError(Exception):
    """Raised when configuration is invalid"""
    pass
