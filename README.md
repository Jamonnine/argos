# ARGOS - ROS 2 Learning Journey

> **A**I-**R**einforced **G**rowth in r**O**botics **S**ystems

A complete beginner-to-practitioner ROS 2 learning path leveraging AI-augmented development workflows.

## 🎯 Mission

Transform from ROS beginner to practical robotics developer in 12-13 weeks through:
- **AI-Pair Programming**: Leverage Claude & GPT for real-time problem solving
- **Project-Based Learning**: 70% hands-on, 30% theory
- **Modern Tooling**: ROS 2 Jazzy, Gazebo Sim → Isaac Sim progression

## 📊 Progress Overview

**Current Phase**: Setup & Initialization
**Week**: 0/13
**Completed Projects**: 0/9

| Phase | Duration | Status | Focus Area |
|-------|----------|--------|------------|
| Phase 1 | Week 1-3 | 🔄 Pending | ROS 2 Fundamentals |
| Phase 2 | Week 4-7 | ⏳ Locked | Gazebo Simulation & Control |
| Phase 3 | Week 8-11 | ⏳ Locked | Vision & Integration |
| Phase 4 | Week 12-13 | ⏳ Locked | Isaac Sim Advanced |

## 🗂️ Repository Structure

```
ARGOS/
├── learning-log/          # Daily learning records & reflections
├── projects/              # Hands-on project implementations
│   ├── week01-first-node/
│   ├── week02-multi-node/
│   ├── week04-mobile-sim/
│   └── ...
├── resources/             # Learning materials & references
├── scripts/               # Setup & utility scripts
├── docs/                  # Documentation & guides
└── README.md
```

## 🚀 Quick Start

### 1. Environment Setup (Ubuntu 24.04 + ROS 2 Jazzy)

```bash
# Run automated setup script
./scripts/setup-ros2-jazzy.sh

# Verify installation
ros2 run demo_nodes_cpp talker
```

### 2. Start Week 1

```bash
# Navigate to week 1 learning plan
cd learning-log/week01/
cat plan.md

# Begin first project
cd ../../projects/week01-first-node/
```

## 📚 Learning Resources

- **Master Plan**: [`docs/learning-plan.md`](docs/learning-plan.md)
- **Daily Templates**: [`learning-log/templates/`](learning-log/templates/)
- **AI Workflow Guide**: [`docs/ai-workflow.md`](docs/ai-workflow.md)
- **Troubleshooting**: [`docs/troubleshooting.md`](docs/troubleshooting.md)

## 🎓 Milestones

### Phase 1 Completion Criteria
- [ ] Write Publisher/Subscriber nodes from scratch
- [ ] Launch multi-node systems with launch files
- [ ] Visualize data in RViz2
- [ ] Solve 3+ errors independently using AI tools

### Phase 2 Completion Criteria
- [ ] Simulate custom robot in Gazebo
- [ ] Implement autonomous navigation with Nav2
- [ ] Control robot arm with MoveIt 2
- [ ] Author/modify URDF files

### Phase 3 Completion Criteria
- [ ] Build vision-based object recognition
- [ ] Complete warehouse automation project
- [ ] Publish project to GitHub with tests
- [ ] Implement CI/CD pipeline

### Phase 4 Completion Criteria
- [ ] Run simulations in Isaac Sim
- [ ] Migrate Gazebo project to Isaac Sim
- [ ] Generate synthetic training data
- [ ] Master remote GPU development (Brev)

## 🛠️ Development Environment

- **OS**: Ubuntu 24.04 LTS (WSL2/Native)
- **ROS**: ROS 2 Jazzy
- **Simulators**: Gazebo Sim → Isaac Sim
- **IDE**: VS Code + ROS Extensions
- **AI Tools**: Claude Code CLI, ChatGPT
- **Version Control**: Git + GitHub

## 📈 Learning Methodology

### Daily Routine (Full-time: 6-8 hours)

**Morning (3-4h)**: Concept Learning
- Official documentation
- Tutorial walkthroughs
- AI-assisted concept exploration

**Afternoon (3-4h)**: Project Development
- Apply learned concepts
- AI pair programming
- Build tangible systems

**Evening (1-2h)**: Review & Documentation
- Update MEMORY.md
- Record lessons learned
- Plan next day

### AI-Augmented Learning Process

1. **Before Coding**: Design review with Claude
2. **During Coding**: Real-time assistance via Claude Code CLI
3. **After Coding**: Code review & best practices validation
4. **Problem Solving**: Context + logs → AI analysis → solution

## 🎯 Key Projects

1. **Week 1-2**: First ROS Node & Multi-Node System
2. **Week 4**: Mobile Robot Simulation (Gazebo)
3. **Week 5-6**: Autonomous Navigation Robot
4. **Week 7**: Robot Arm Pick & Place
5. **Week 8**: Vision-Based Object Recognition
6. **Week 9-11**: **Warehouse Automation System** (Portfolio Centerpiece)
7. **Week 12-13**: Isaac Sim Final Project

## 📞 Community & Support

- **ROS Discourse**: https://discourse.ros.org/
- **Reddit**: r/ROS
- **GitHub**: Star & follow ROS 2 repositories

## 📝 License

This learning repository is personal educational material. Code projects are MIT licensed unless otherwise specified.

---

**Last Updated**: 2026-02-08
**Current Focus**: Environment setup & Week 1 preparation
**Next Milestone**: First ROS 2 node execution
