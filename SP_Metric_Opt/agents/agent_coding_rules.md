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
- Leave stages for code refactor

Working with commit history:
- Agenst only run `git add`, then ask user for review. Only users can run `git commit`
- Only add relevant changes to a commit's core purpose, don't add file chagnes that are not related to a commit's goal
- Ensure all tests pass before handling code review to users

Code refactor principles:
- Reduce repaeted code
- Foster modular design
- Don't consider backward compatibility unless explicitly specified by users

Making plans:
- First decompose the main task into high-level steps
- Within each high-level steps, iteratively, find one minimal, small, modular modification change that you can make to make the current code base one step closer to achieving the high-level step goal. The code change must be testable, and all tests must be able to pass after each small step modification. Follow test-driven development to consider what would be good test cases.
- Keep this iteration loop til the step-level goal is accomponished.


Code readabiliy is important
- Don't use very short names or names with only a few letters such as "K" or "tl", long names are okay.
- Use short functions. If a function has to be very long, try to extract sub-functions for logic that hold together.

Tests
- If you modified any C++ code, run `cd build && make check.SP_OPT -j5` in build folder for functionality testing, run `cd release && make -j5 && ./tests/RunSpeedTest` for speed tests.

Variable names (during coding and chat)
- Use informative names, especially during chat. Variable names must be clear even without context, bad examples include `ndiff` or `nvar`, good examples could be `num_tasks_with_diff_et`

Tasks and planning:
- Assign priority level to tasks, from P0 P1 P2 P3

Start new tasks:
- follow examples in `agents/active_tasks` folder to create new task

Work with users:
- Give user complete context when asking users to make decisions, don't use abbrevation too much. Especially never make up new terminologies or new concepts or new abbrevations.

Ask questions:
don't ask questions to ask questions or for user confirmation. if you can make decisions, you can take it. othewise, provide enough context to users to decide.

Variable Names:
Typically, use positive phase rather than negative phase, unless the context prefers a negative phase. For example, use "important_tasks_schedulability_check" rather than "no_important_task_gate".

code speaks for itself without comments:
never let comments do the work, always make the code work and readable and clear even without any comments. I hate comment paragraphs, don't add a paragraph of comments or comments with many lines (more than 5 lines), ideally, make comments less than 3 lines, if any.
Do not add comments that line-to-line explain code logic, code speaks for itself, those comments waste token, useless, and no human read it. Ideally, function names speak for the purpose of a function implementation / goal.

Code commit rule:
Short, Simple, Modular, Multiple. Each commit should contain minimum modular changes that involve no more than, ideally, 3 source file changes, there are no limits in test file changes. We can add many committs for one task.