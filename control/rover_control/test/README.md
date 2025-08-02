# Rover Control Testing

This directory contains tests for the rover_control package, following a phase-gated testing approach for safety and reliability.

## Testing Phases

### Phase 1: Unit Tests (Safe)
- Test individual components in isolation
- Mock all hardware interfaces
- Validate configuration loading
- Test safety monitoring algorithms
- Test pure pursuit path following logic

### Phase 2: Integration Tests (Mock Hardware)
- Test component interactions
- Validate action server workflows
- Test safety monitor integration
- Test path following with mock sensors
- Test launch file sequencing

### Phase 3: Hardware-in-Loop Tests (Real Hardware)
- Test with actual motor controllers
- Validate encoder feedback
- Test emergency stop systems
- Test CAN bus communication
- Validate safety response times

### Phase 4: Field Tests (Competition Environment)
- Test in outdoor conditions
- Validate GPS integration
- Test visual servoing
- Test stuck detection and recovery
- Test bandwidth monitoring

## Test Structure

```
test/
├── unit/
│   ├── test_safety_monitor.py      # Safety monitoring unit tests
│   ├── test_path_follower.py       # Path following algorithms
│   ├── test_action_servers.py      # Action server logic
│   └── test_hardware_interface.py  # Hardware interface mocking
├── integration/
│   ├── test_system_startup.py      # Launch sequence validation
│   ├── test_navigation_workflow.py # End-to-end navigation
│   └── test_safety_integration.py  # Safety system integration
├── hardware/
│   ├── test_motor_control.py       # Motor controller validation
│   ├── test_encoder_feedback.py    # Encoder accuracy tests
│   └── test_emergency_stop.py      # E-stop response timing
└── field/
    ├── test_gps_accuracy.py        # GPS positioning accuracy
    ├── test_visual_servoing.py     # Tennis ball approach
    └── test_competition_workflow.py # Full competition simulation

```

## Running Tests

### Unit Tests (Always Safe)
```bash
# Run all unit tests
python -m pytest test/unit/ -v

# Run specific test file
python -m pytest test/unit/test_safety_monitor.py -v

# Run with coverage
python -m pytest test/unit/ --cov=rover_control --cov-report=html
```

### Integration Tests (Mock Hardware Required)
```bash
# Launch mock system first
ros2 launch rover_control rover_control.launch.py mock_hardware:=true

# Run integration tests
python -m pytest test/integration/ -v
```

### Hardware Tests (Real Hardware Required)
```bash
# ⚠️  WARNING: Only run with real hardware setup
# Ensure safety systems are active and tested first

ros2 launch rover_control rover_control.launch.py mock_hardware:=false

# Run hardware tests (use with caution)
python -m pytest test/hardware/ -v
```

### Field Tests (Competition Environment)
```bash
# ⚠️  WARNING: Only run in safe, open field environment
# Ensure all safety systems are validated

# Full system test
python -m pytest test/field/ -v --tb=short
```

## Test Configuration

### Mock Hardware Setup
- Uses simulated motor controllers
- Provides fake sensor data
- Simulates network conditions
- Safe for continuous integration

### Safety Test Requirements
- All tests must validate emergency stop functionality
- Safety monitor must be active during all tests
- Hardware tests require manual safety oversight
- Field tests require spotter and emergency stop access

## Continuous Integration

### Automated Tests (CI/CD)
- Unit tests run on every commit
- Integration tests run on pull requests
- Mock hardware tests run nightly
- Hardware tests run manually before releases

### Test Coverage Requirements
- Minimum 80% code coverage for safety-critical components
- 100% coverage for fault detection algorithms
- Integration tests for all action workflows
- Performance tests for real-time requirements

## Safety Testing Protocol

### Emergency Stop Testing
1. Verify software emergency stop (< 150ms response)
2. Verify hardware emergency stop (immediate)
3. Test emergency stop during all motion states
4. Validate fault recovery procedures

### Fault Injection Testing
1. Simulate motor failures
2. Simulate communication timeouts
3. Simulate sensor failures
4. Test stuck detection and recovery

### Performance Testing
1. Validate 50 Hz control loop timing
2. Test safety monitor 100 Hz response
3. Measure action server latency
4. Validate path following accuracy

## Test Data and Metrics

### Performance Benchmarks
- Control loop jitter: < 2ms
- Safety response time: < 150ms
- Path following accuracy: < 10cm RMS
- Goal reaching accuracy: < 5cm

### URC-Specific Tests
- Autonomy timer validation (60s)
- Bandwidth monitoring (5 Mbps limit)
- Tennis ball approach accuracy (< 5cm)
- Stuck detection sensitivity tuning

## Development Workflow

### Before Committing Code
1. Run all unit tests
2. Run relevant integration tests
3. Validate configuration changes
4. Update test documentation

### Before Hardware Testing
1. All mock tests must pass
2. Safety systems verified
3. Hardware setup checklist completed
4. Emergency procedures reviewed

### Before Competition
1. All test phases completed
2. Field testing validation
3. Performance benchmarks met
4. Backup system tested

## Test Environment Setup

### Dependencies
```bash
# Install test dependencies
pip install pytest pytest-cov pytest-mock
pip install numpy scipy matplotlib  # For test data analysis

# Install ROS 2 testing tools
sudo apt install ros-humble-test-* ros-humble-launch-testing
```

### Mock Hardware Configuration
```yaml
# test_config.yaml
mock_hardware:
  enable: true
  simulate_delays: true
  inject_noise: true
  simulate_faults: false

test_scenarios:
  basic_motion: true
  path_following: true
  obstacle_avoidance: true
  emergency_stop: true
```

## Troubleshooting

### Common Test Failures
- **Timing tests fail**: Check system load, disable other processes
- **Mock hardware issues**: Verify test configuration, restart nodes
- **Integration test failures**: Check node startup sequence
- **Hardware test errors**: Verify connections, check safety systems

### Debug Commands
```bash
# Check node status
ros2 node list
ros2 topic list
ros2 service list

# Monitor test topics
ros2 topic echo /rover/safety_status
ros2 topic echo /rover/status

# Check timing
ros2 run rover_control hardware_interface --ros-args --log-level debug
```

For questions or issues, consult the main rover_control documentation or contact the development team. 