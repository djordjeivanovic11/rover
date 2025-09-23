# ARM CONTROL TEST SUITE

**⚠️ IMPORTANT: DO NOT RUN TESTS UNTIL CORRESPONDING PHASE IS COMPLETE ⚠️**

This test suite provides comprehensive testing for all arm control components. Each test is gated by implementation phase to prevent running tests on incomplete code.

## Test Phases

### Phase 1: Individual Component Tests ✅ READY
- Hardware interface (mock mode)
- Safety monitor (simulated data)
- Configuration loading and validation

### Phase 2: Integration Tests 🚫 DO NOT RUN
- Component communication
- Safety system integration
- Real-time performance validation

### Phase 3: System Tests 🚫 DO NOT RUN  
- End-to-end action execution
- Mission scenario testing
- Hardware-in-the-loop testing

### Phase 4: Performance Tests 🚫 DO NOT RUN
- Timing validation (250Hz, <150ms response)
- Load testing
- Stress testing

## Running Tests

### Prerequisites
```bash
# Ensure all dependencies are installed
sudo apt update
sudo apt install ros-humble-control-msgs ros-humble-trajectory-msgs

# Build the package
cd /path/to/workspace
colcon build --packages-select arm_control

# Source the workspace  
source install/setup.bash
```

### Phase 1 Tests (Safe to run)
```bash
# Test configuration loading
python3 -m pytest test/test_configuration.py -v

# Test hardware interface (mock mode only)
python3 -m pytest test/test_hardware_interface.py::TestMockHardware -v

# Test safety monitor (simulated data)
python3 -m pytest test/test_safety_monitor.py::TestSimulatedSafety -v
```

### Phase 2 Tests (DO NOT RUN YET)
```bash
# Integration tests - ONLY run after Phase 1 complete
# python3 -m pytest test/test_integration.py -v
```

### Phase 3 Tests (DO NOT RUN YET)
```bash
# System tests - ONLY run after Phase 2 complete  
# python3 -m pytest test/test_system.py -v
```

### Phase 4 Tests (DO NOT RUN YET)
```bash
# Performance tests - ONLY run after Phase 3 complete
# python3 -m pytest test/test_performance.py -v
```

## Test Descriptions

### test_configuration.py ✅
- Validates YAML configuration loading
- Checks parameter consistency across files
- Tests default value fallbacks
- **Safe to run**: Uses only static configuration files

### test_hardware_interface.py ⚠️
- `TestMockHardware` ✅: Tests mock hardware interface
- `TestRealHardware` 🚫: Tests real hardware (requires hardware)
- **Mock tests safe**: No hardware interaction

### test_safety_monitor.py ⚠️  
- `TestSimulatedSafety` ✅: Tests with simulated sensor data
- `TestRealSafety` 🚫: Tests with real sensor feeds
- **Simulated tests safe**: Uses mock data publishers

### test_trajectory_executor.py 🚫
- Tests trajectory streaming and monitoring
- Requires hardware interface to be running
- **DO NOT RUN**: Needs Phase 1 complete

### test_gripper_controller.py 🚫
- Tests force-based grasping
- Requires gripper hardware or simulation
- **DO NOT RUN**: Needs Phase 1 complete

### test_tool_manager.py 🚫
- Tests tool change operations
- Requires MoveIt integration
- **DO NOT RUN**: Needs Phase 2 complete

### test_action_servers.py 🚫
- Tests high-level action interfaces
- Requires all lower-level components
- **DO NOT RUN**: Needs Phase 3 complete

### test_integration.py 🚫
- Tests component communication
- Requires multiple nodes running
- **DO NOT RUN**: Needs Phase 1 complete

### test_system.py 🚫
- End-to-end mission scenarios
- Requires full system integration
- **DO NOT RUN**: Needs Phase 2 complete

### test_performance.py 🚫
- Timing and real-time performance
- Requires stable system
- **DO NOT RUN**: Needs Phase 3 complete

## Test Data

### Mock Sensor Data
- Joint positions: `test/data/mock_joint_states.yaml`
- Force/torque: `test/data/mock_wrench_data.yaml`
- Temperature/current: `test/data/mock_sensor_data.yaml`

### Test Trajectories
- Simple motions: `test/data/test_trajectories.yaml`
- Pick/place scenarios: `test/data/manipulation_tests.yaml`

### Safety Test Cases
- Fault scenarios: `test/data/fault_test_cases.yaml`
- Limit violations: `test/data/safety_violations.yaml`

## Continuous Integration

### GitHub Actions (Future)
```yaml
# .github/workflows/test.yml
- name: Phase 1 Tests
  run: python3 -m pytest test/test_configuration.py test/test_hardware_interface.py::TestMockHardware

# Other phases gated by manual approval
```

### Pre-commit Hooks
```bash
# Only run safe tests
pre-commit run --hook-stage manual phase1-tests
```

## Adding New Tests

### Guidelines
1. **Always gate tests by phase**
2. **Mark unsafe tests clearly**  
3. **Provide mock data for hardware tests**
4. **Include teardown for cleanup**
5. **Document hardware requirements**

### Test Template
```python
import pytest
from arm_control.test.utils import requires_phase, mock_hardware

class TestNewComponent:
    
    @requires_phase(1)
    def test_configuration_loading(self):
        """Safe test - configuration only"""
        pass
    
    @requires_phase(2)  
    @mock_hardware
    def test_with_mock_hardware(self):
        """Phase 2 test with mock hardware"""
        pass
    
    @requires_phase(3)
    @pytest.mark.hardware
    def test_with_real_hardware(self):
        """Phase 3 test - requires real hardware"""
        pass
```

## Troubleshooting

### Common Issues
1. **ModuleNotFoundError**: Source workspace and rebuild
2. **roscore not running**: Tests use pytest-ros fixtures
3. **Permission denied**: Check hardware device permissions
4. **Timeout errors**: Increase test timeouts for slower systems

### Debug Mode
```bash
# Run tests with debug output
python3 -m pytest test/ -v -s --log-cli-level=DEBUG
```

### Test Coverage
```bash
# Generate coverage report
python3 -m pytest test/ --cov=arm_control --cov-report=html
```

## Safety Notes

⚠️ **CRITICAL SAFETY REMINDERS**:

1. **Never run hardware tests without proper safety setup**
2. **Always use E-stop when testing with real hardware**  
3. **Start with mock mode and simulated data**
4. **Verify safety limits before any motion**
5. **Have manual override procedures ready**

## Support

For test issues or questions:
- Check test logs: `test/logs/`
- Review configuration: `config/`
- Contact team: urc-arm-team@rover.com 