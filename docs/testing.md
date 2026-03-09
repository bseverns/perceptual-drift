# Testing Guide

Use this page as the practical map for what each test layer covers, what it
needs, and how to run it in local/CI-like environments.

## Test taxonomy

| Layer | Test files | What it validates | Environment needs |
| --- | --- | --- | --- |
| Pure/unit logic | `tests/test_swarm_hardening.py` | Swarm behavior math, collision helpers, replay interpolation | No sockets, no hardware |
| Config/schema validation | `tests/test_validate_config.py` | Mapping + recipe validation failure/success behavior | Filesystem only |
| Operator state logic | `tests/test_operator_ui_state.py` | Recipe/consent state transitions, dispatch history, exports, health snapshot logic | Filesystem + mocked dependencies |
| Supervisor lifecycle | `tests/test_operator_ui_supervisor.py` | Runtime start/stop/status orchestration and option parsing | Local subprocess execution |
| Bridge integration harness | `tests/test_check_stack.py` | OSC fixture replay, bridge mapper behavior, MSP framing, consent neutralization paths | UDP socket bind allowed |
| Operator API auth/integration | `tests/test_operator_ui_server.py` | HTTP auth gate behavior (`/api/health` public, others token-protected) | TCP socket bind allowed |

## Run commands

From repo root:

```bash
python3 -m pip install --upgrade pip setuptools wheel
python3 -m pip install -r requirements-dev.txt -c constraints/py310-linux.txt
python3 -m pip install -e . --no-build-isolation
```

Full suite:

```bash
python3 -m pytest tests -q
```

Targeted runs:

```bash
python3 -m pytest tests/test_validate_config.py -q
python3 -m pytest tests/test_operator_ui_state.py -q
python3 -m pytest tests/test_check_stack.py -q
python3 -m pytest tests/test_operator_ui_server.py -q
```

CI parity checks:

```bash
flake8 --max-line-length 110 --extend-ignore E203 \
  software/control-bridge \
  software/operator_ui \
  scripts/check_stack.py \
  scripts/validate_config.py \
  tests
black --check \
  software/control-bridge \
  software/operator_ui \
  scripts/check_stack.py \
  scripts/validate_config.py \
  tests
python3 -m pytest tests -q
pd-check-stack --max-frames 24 --send-interval 0.01 --cooldown 0.05
```

## Socket-binding caveat

Some tests intentionally start local servers (`127.0.0.1` + ephemeral ports).
In restricted sandboxes, these can fail with `PermissionError: [Errno 1]
Operation not permitted`.

If that happens:

- Re-run tests in an environment that allows local TCP/UDP bind.
- Or run non-socket subsets while iterating:

```bash
python3 -m pytest tests -k "not check_stack and not operator_ui_server" -q
```

## Contribution guidance

When adding tests:

- Prefer deterministic inputs and fixed seeds for replay/mapping behavior.
- Keep hardware out of the default test path; use mocks/stubs.
- Use `port=0` for ephemeral server ports in integration tests.
- Keep assertions tied to behavioral contracts (consent gating, neutralization,
  auth decisions), not incidental log text.
