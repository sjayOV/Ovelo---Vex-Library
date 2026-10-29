# Design decisions and reference projects

Ovelo's design goal is to make an autonomous stack inspectable by a learner. Its independent implementation combines a small portable core, explicit input contracts, a hand-motion diagnostic, a trace of each odometry calculation, and motion results that explain why a command stopped.

These are useful design choices rather than a claim that nobody else has used them. Standard odometry equations, proportional control, and particle filtering are established techniques. Claims of superior accuracy, speed, simplicity, adoption, or globally unique algorithms require evidence this project does not have.

## Reference resources

| Resource | Use it for |
| --- | --- |
| [LemLib source](https://github.com/LemLib/LemLib) and [documentation](https://lemlib.readthedocs.io/) | An established PROS chassis/odometry ecosystem; compare current setup, tracking-wheel conventions, tuning, and motion behavior |
| [EZ-Template](https://github.com/EZ-Robotics/EZ-Template) | An established VEX PROS template and practical robot configuration/tuning examples |
| [PROS documentation](https://pros.cs.purdue.edu/v5/) and [kernel source](https://github.com/purduesigbots/pros) | Authoritative SDK installation, APIs, lifecycle, units, errors, and supported V5 operations |

The environment's network proxy was unreachable during implementation, so these resources could not be retrieved or checked against a current revision. They are references for the user and future verification, not a claim of having inspected their current source. No third-party implementation was copied into Ovelo. If incorporating third-party code later, preserve its license, attribution, and applicable notices.

## Why hardware is separated

The core has no PROS include or motor constructor. This allows desktop tests of angle wrap, sensor offsets, fault handling, and simulated motions. The adapter translates actual PROS readings into the core's units and applies motor voltage.

This separation also creates a clear verification boundary. A successful desktop test cannot detect a changed SDK API, a link problem, reversed wiring, or a wheel that loses contact. The V5 adapter must compile against a real kernel and then pass physical checks.

## Ovelo versus adopting another template

Choose Ovelo when you want to study a compact implementation, inspect intermediate calculations, or experiment with a hardware-independent mathematical core. Consider LemLib or EZ-Template directly when their documented features, ecosystem, and existing robot experience better fit your project. Their current documentation should establish their capabilities; Ovelo does not make a feature-by-feature superiority claim.

Ovelo's controller deliberately has fewer capabilities than a full navigation framework. It does not follow splines or paths, avoid obstacles, discover motor/sensor wiring, tune itself, or fuse particle estimates into odometry automatically. Its optional wall model assumes an empty rectangular field; it cannot model arbitrary game objects or moving robots.

## Authorship and provenance

The starting attachments supplied a README, changelog, Makefile, license, and ignore file, with no prior implementation. This version reconstructs those described features with independently written code and AI assistance. The supplied MIT license attribution is retained. Review the code and measurements before release, and follow the current policies for the event where the work will be used.
