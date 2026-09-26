# Copyright 2026 huudinh
# Licensed under the MIT License

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Check source code for docstring conformance."""
    rc = main(argv=[])
    assert rc == 0
