# Ch.22 — Closing: Where to Start?

Twenty-one chapters behind us. How sensors turn the world into numbers, how those numbers flow through coordinate frames, how a robot moves on top of that, and where deep learning fits into this pipeline — one full loop. The remaining question is "so what do I do starting tomorrow?"

## 22.1 The Map So Far

Four parts.

- **Foundations** (Ch.1–3): the name Spatial AI, how sensors capture the world, and the math to handle it. The rotations, transformations, optimization, and probability introduced here came back in every later chapter.
- **Robots** (Ch.4–8): how to handle a physical system of joints and links. Kinematics solves pose, dynamics computes forces, control drives it to the desired state, motion planning lays out paths, and learning picks up the whole thing from data.
- **Perception and Spatial Understanding** (Ch.9–14): starting from images and climbing up to 3D space. Deep learning was stacked on top of classical CV, foundation models on top of that, and attempts like VLA that bind language and action follow. SLAM places this whole stack on a time axis.
- **Research in Practice** (Ch.15–21): what you need to actually run code. Frameworks, tools, datasets, and how to survive in a lab.

These four parts are not laid out to be read in order — they are closer to four rooms you can return to when needed. In practice, moments come when, standing in front of a robot, you have to use the equations of Ch.3, the SLAM of Ch.14, and the Docker of Ch.16 all at once.

## 22.2 Starting Points by Profile

The question I hear most often is "where do I start reading?" The door you enter depends on your background.

If you are a **third- or fourth-year undergraduate new to robotics**, I recommend Ch.1 → Ch.3 → Ch.9 → Ch.14. Get a feel for what Spatial AI is, pick up the mathematical language, climb from images to 3D, and then see how the pieces click together in SLAM. If you set up the practice environment of Ch.16 along the way, you can start moving your hands right away.

Someone arriving as a **new master's student with a deep learning background** should read Ch.2 → Ch.3 → Ch.10 → Ch.11 first. Starting from the language you already know (deep learning), learn the new grammar of sensors and math, and then see how foundation models enter robotics. After that, move to Ch.14 and Ch.13 for the 3D and SLAM side.

Those with a **classical robotics background who are weak on deep learning** should head straight to Ch.8 → Ch.10 → Ch.11 → Ch.12. Skim Ch.4–7, which you already know, and focus on what changed over the past five years. By the time you reach VLA (Ch.12), you can see where the field's center of gravity has shifted.

**Someone with a concrete project** goes in reverse. Start by skimming the datasets and benchmarks of Ch.17, pick one or two papers on a similar task, and trace backward only to the chapters those papers depend on. There is no need to read everything.

Ch.20.7's learning roadmap has more detailed recommendations by timeframe (1 month, 3 months, 6 months). This chapter only sets the broad direction.

## 22.3 What Not to Do

A few traps newcomers fall into most often, from experience.

**Trying to read many papers first.** Without background, stacking up twenty papers leaves you with twenty disconnected papers. Grabbing one paper and reading the three chapters it references beats skimming ten. In the first month especially, slowly going through this guide once is faster.

**Trying to perfect the environment first.** Insisting Docker be perfect, the GPU set up, and the dataset downloaded before starting eats up two to three weeks. Copy a senior's environment, change one line in code that already runs — that is faster. Setup can be learned when problems show up.

**Trying to write everything yourself.** As of 2026, ORB-SLAM3, Colmap, and Gaussian Splatting are all open source. Writing it yourself only makes sense for educational purposes or when there is a genuinely new contribution. In an unfamiliar area, running an existing implementation first and finding where it breaks is closer to research.

**Skipping fundamentals because "AI will write it for me".** Code agents move fast in the direction you design, but if the direction is wrong, they move fast in the wrong direction. Without the skeleton of the equations and algorithms, you cannot judge whether the agent's suggestion is correct. The workflow in Ch.19 also only works well "when the user can judge".

## 22.4 A Sense of the Long Game

Robotics is not a field for the impatient. The first paper takes a year and a half. Once experiments start running, results come within two weeks, but getting the experiments to run takes three to six months. Knowing this makes you less shaken by day-to-day pace.

From the perspective of three to four years of graduate school, the first six months are **time to learn the tools**. Some of ROS, PyTorch, Git, paper reading, MATLAB, and CAD have to feel natural in your hands. The next six months are **time to run and break existing systems**. Experience of reading someone else's code, failing, and fixing it accumulates. Around the year-and-a-half mark, your own problem finally appears. That is when research begins.

When this order gets flipped — trying to look for "new ideas" before ever running someone else's code — things usually stall. If you do not know why the existing system looks the way it does, it is also hard to explain why a new proposal is needed.

## 22.5 One Line

If you have read this far, you already have one ability — the ability to read a long text to the end. Agents write code these days, but reading a ten-thousand-line document and grasping the context is still a human job. People who read well also wield AI well.

When stuck, Slack. This document keeps getting rewritten.

---

*Questions or feedback: the lab Slack channel.*

Draft date: 2025.12.28 · Revision date: 2026.04.23
