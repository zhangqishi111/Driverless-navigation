import landerpi_sandbox_display.sandbox_display_node as display_module


def test_legacy_sandbox_window_is_removed():
    assert not hasattr(
        display_module,
        'SandboxWindow',
    )