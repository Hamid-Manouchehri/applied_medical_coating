# Codex Guidelines

Use this file as the project-level source of truth for how Codex should write, edit, explain, and verify Python code in this directory.

At the start of a new session, say:

```text
Read CODEX_GUIDELINES.md first and follow it for this project.
```

As a starter for this chat remember; I am using this for mechanical engineeirng robotics/research work (not computer science / engineer level coding). I do not want fancy engineering patterns unless necessary. Prefer simplest readable explicit code that I can inspect and modify myself. Optimize for understanding first, speed second. Avoid unnecessary abstractions, clever one-liners, and overengineering. Explain (shortly) the logic before and after implementation. Keep formulas, assumptions, units, and input/output behavior visible. Also, create a section for "User Settings". add a "TODO" comment with very short comment if needed in front of the each line which is prone to change by user. No need for unneccessary "guard clause" "validation check", or "error handling", first version of programs must be very simple and easy to understand.

Note: Make the code understandable in such a way if I opened after 3 weeks or so, I can figure out what it is doing ---> high-level understanding

## Project Context

- Project name: physical human-robot interaction (pHRI)
- Main goal of this project: Data analysis, statistical analysis, and feature extraction (e.g. 2/3 powerlaw fit) from human-subject study in human robot co-manipulation.
- Operating system/environment: Ubuntu 24
- Code that should not be changed without asking: -

## Python Coding Style

- Preferred script/function organization: [FILL-IN: e.g., one main function per file, local helper functions at bottom, scripts only for quick experiments]
- Function naming style: snake_case and self-explanatory
- Variable naming style: self-explanatory, for the input files variable names with "read_RELEVANT_NAME" and output file variable names with "write_WHATEVER_RELEAVANT", if it is both input and output then "read_write_WHATEVER_RELEVANT".
- Constant naming style: CAPITAL LETTERS
- File naming style: self-explanatory
- Maximum line length: 76 letters

## Python Function Structure

Preferred function template:

```python
def function_name(args):
    """Short one-line summary.

    Longer explanation if needed.

    Inputs:
        input: [FILL-IN expected type/shape/units]

    Outputs:
        output: [FILL-IN expected type/shape/units]
    """

    # User Settings
    # TODO: input_path = "path/to/input.csv"

    # Main logic here.

```

## Comments And Documentation

- Header comments required for functions: Create a short understandable explanation of each programming file within its own file, including, what input and output is and other necessary stuffs required for getting an overview of the file code.
- Input/output documentation style: yes
- Explain formulas with references: yes
- Include examples in function comments: no


## Testing And Verification

- Preferred testing approach: Start with simple example scripts and `assert` checks. Use formal `pytest` tests only when the project grows.
- Test file naming/location: Put simple test scripts in a `tests/` folder when useful.
- Minimum checks before finishing: Run at least one small example with known expected output, check for errors, and verify important edge cases.
- When tests are unavailable, verify by: Run the main script/function on a small sample input and inspect plots/output for obvious errors.
- Should Codex run Python commands if available: Ask first.


<!-- ## Dependencies

- Python version: [FILL-IN]
- Allowed Python packages: [FILL-IN]
- Packages to avoid: [FILL-IN]
- External dependencies allowed: [FILL-IN: yes/no/ask first]
- C/C++ extension integration allowed: [FILL-IN] -->

## Editing Rules For Codex

<!-- - Before editing, Codex should: inspect relevant files and summarize intended changes -->
- Ask before adding new files: yes
- Ask before changing public APIs: yes
- Ask before deleting code/files: yes
- Preserve existing user edits: yes

## Explanation Style

- Preferred level of explanation: first implement what I asked, then if I had question I will ask you or if something is like requires explanation, explain it.


## Personal Preferences

- Things I like in code: readability
- Things I dislike in code: data validation
- Common mistakes Codex should watch for: -
- My preferred workflow with Codex: -
- Anything else Codex should remember for this project: -

<!-- ## Current Project Notes

- Current task/direction: [FILL-IN]
- Important decisions already made: [FILL-IN]
- Open questions: [FILL-IN]
- Next tasks: [FILL-IN] -->
