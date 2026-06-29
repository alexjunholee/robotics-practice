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

Ch.20.7's learning roadmap has more detailed recommendations by timeframe (1 month, 3 months, 6 months). Here the goal is only to set the broad direction.

## 22.3 What Not to Do

The four traps newcomers fall into most often — *reading many papers first / perfecting the environment first / writing everything yourself / skipping fundamentals because "AI will write it"* — are treated generally in [Research Notes Ch.6](../../research-notes/guide.html#ch6-왜-읽는가-그리고-무엇을-안-읽을지-정하는-법) (the trap of reading-many) and [Grad Notes Ch.11](../../grad-notes/guide.html#ch11-도구의-함정-장비병과-가짜-진척) (the tool-fetish trap — environment / from-scratch / AI dependency) *(Korean; English version planned)*.

Field-specific application in one line. SLAM/CV has *deep field tooling* — ORB-SLAM3, Colmap, and Gaussian Splatting are open-source standards. Implementing from scratch is meaningful only for educational purposes or when there is a genuinely new contribution. Otherwise, running an existing implementation and finding where it breaks is closer to research.

## 22.4 A Sense of the Long Game

The PhD-time frame (the first year, the year-and-a-half mark, five years out) is treated in the meta-skill guide — [Grad Notes Ch.7](../../grad-notes/guide.html#ch7-내-연구를-갖기-0년차-학생에서-5년차-연구자로) (when your own research appears) and [Grad Notes Ch.1](../../grad-notes/guide.html#ch1-박사를-결정한다는-일) (the time-horizon of the PhD decision) *(Korean; English version planned)*.

Robotics experiments commonly run on a *3–6 month setup, 2-week experiment-cycle* unit. The year-and-a-half first-paper estimate comes from the same place. When the field's time-sense merges with the PhD's operating frame, daily pace shakes you less.

## 22.5 One Line

If you have read this far, you already have one ability — the ability to read a long text to the end. The field core ends here, in one full loop.

The full guide on PhD operation and reading mindset lives at [`../../research-notes/`](../../research-notes/)·[`../../grad-notes/`](../../grad-notes/) *(Korean; English version planned)*.

Draft date: 2025.12.28 · Revision date: 2026.05.01
