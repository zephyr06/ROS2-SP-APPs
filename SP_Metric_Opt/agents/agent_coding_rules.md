Generatl coding rules:
- Follow test-driven development, first add test cases that existing code base fails, then work on the code to fix them, finally refactor code for user review
- Reduce usage of optional arguments, as that is error-prone. Users may easily forget to pass certain configurations. If something important is needed but not passed, raise an error to request the configurations
- Work by module, commit by module. Foster modular design and development
- Regularly update dev_log.md and tasks.md, whenever you make certain progress. Agent converations can be lost very easily.
- Modular development: work on a small sub task each time, and ask user to review and commit after done one small sub task.

Design rules:
- We don't consider backward compatibility and ruthlessly prune features that are not used anymore.
- Keep things simple, do not make things optional if not needed
- Ask users if you're not certain about design choices, don't make design decisions yourself.

Working with commit history:
- Agenst only run `git add`, then ask user for review. Only users can run `git commit`
- Ensure all tests pass before handling code review to users

Code refactor principles:
- Reduce repaeted code
- Foster modular design
- Don't consider backward compatibility unless explicitly specified by users
