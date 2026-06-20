import pytest
from Gen_Taskset.lib.yaml_exporter import format_list_to_space_separated_string

def test_format_list_to_space_separated_string():
    data = [1.2356, 2.0, 3]
    result = format_list_to_space_separated_string(data)
    assert result == "1.236 2.000 3"
