class RecipeValidationError(Exception):
    """Raised for any structurally or semantically invalid recipe (R1: reject
    malformed input cleanly rather than crashing)."""
