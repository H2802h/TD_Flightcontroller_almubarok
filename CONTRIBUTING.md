# Contributing to TD Flight Controller

First off, thank you for considering contributing to TD Flight Controller! It's people like you that make this project better.

## Code of Conduct

By participating in this project, you are expected to uphold respectful and professional behavior.

## How Can I Contribute?

### Reporting Bugs

Before creating bug reports, please check existing issues to avoid duplicates. When creating a bug report, include as many details as possible:

**Bug Report Template:**
```markdown
**Describe the bug**
A clear description of what the bug is.

**To Reproduce**
Steps to reproduce the behavior:
1. Configure with '...'
2. Fly in '...'
3. Observe '...'

**Expected behavior**
What you expected to happen.

**Hardware Setup:**
- Board: [e.g., Arduino Mega]
- IMU: [e.g., MPU6050]
- Frame: [e.g., 450mm quadcopter]

**Software Version:**
- Firmware version: [e.g., v1.0]
- Arduino IDE: [e.g., 1.8.19]

**Additional context**
Add any other context, logs, or screenshots.
```

### Suggesting Enhancements

Enhancement suggestions are tracked as GitHub issues. When creating an enhancement suggestion, include:

- Clear and descriptive title
- Detailed description of the proposed functionality
- Explanation of why this would be useful
- Possible implementation approach

### Pull Requests

1. **Fork the repo** and create your branch from `main`
2. **Write clear code** with comments
3. **Test thoroughly** without propellers first
4. **Update documentation** if needed
5. **Follow the code style** used in the project
6. **Write a clear commit message**

## Development Setup

```bash
git clone https://github.com/yourusername/TD_Flightcontroller_almubarok.git
cd TD_Flightcontroller_almubarok
```

## Coding Conventions

### Code Style
- Use meaningful variable names
- Comment complex algorithms
- Keep functions focused and small
- Use consistent indentation (2 or 4 spaces)

### Naming Conventions
```cpp
// Constants: UPPER_CASE
#define MAX_THROTTLE 2000

// Functions: camelCase
void calculatePID() { }

// Variables: camelCase
float pidOutput;

// Classes: PascalCase
class MotorController { }
```

### Comments
```cpp
// Single line comments for brief explanations

/*
 * Multi-line comments for:
 * - Function descriptions
 * - Complex algorithm explanations
 * - Important notes
 */
```

## Testing Checklist

Before submitting a PR:

- [ ] Code compiles without errors
- [ ] Tested on actual hardware (if possible)
- [ ] No propellers during initial testing
- [ ] Serial output shows expected values
- [ ] IMU calibration works correctly
- [ ] Motor outputs are correct
- [ ] Radio inputs are read properly
- [ ] Failsafes activate appropriately
- [ ] Documentation updated
- [ ] Comments added for new code

## Commit Messages

Write clear commit messages:

```
Short summary (50 chars or less)

More detailed explanation if needed. Wrap at 72 characters.
Explain the problem this commit solves and why this approach
was chosen.

Fixes #123
```

Good commit examples:
- `Add altitude hold feature using barometer`
- `Fix motor direction for X-configuration`
- `Improve PID loop timing accuracy`
- `Update calibration documentation`

## Project Structure

```
src/
├── main/           # Main flight controller code
├── sensors/        # Sensor drivers
├── control/        # PID and flight control
├── radio/          # Radio receiver interface
└── utils/          # Helper functions
```

## Questions?

Feel free to open an issue for questions or join our discussions!
