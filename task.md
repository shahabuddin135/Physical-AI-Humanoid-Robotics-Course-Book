# 🤖 Physical AI & Humanoid Robotics Book - Project Tasks

> **Project Codename:** "Learn TO built a Robot with Dark Humor"  
> **Vibe:** Dark humor, adult jokes, brutally honest, engaging AF  
> **Style Reference:** Pinecone.io - Minimal, dark theme, smooth interactions, beautiful typography

---

## 📋 PHASE 0: Project Setup & Cleanup ✅

### Task 0.1: Clean Up Existing Files
- [x] Remove all example markdown files in `/docs`
- [x] Remove example blog posts in `/blog`
- [x] Clean up default homepage components
- [x] Remove unnecessary assets from `/static/img`

### Task 0.2: Project Structure Planning
- [x] Create `book-structure.md` - Outline of all chapters and sections (in memory.md)
- [x] Create `style-guide.md` - Writing style, tone, humor guidelines (in memory.md)
- [x] Create `chunking-strategy.md` - RAG-optimized content structure for embedding (in memory.md)

---

## 📋 PHASE 1: Design System & Landing Page ✅

### Task 1.1: Docusaurus Theme Configuration
- [x] Configure `docusaurus.config.ts` for dark mode by default
- [x] Set up custom color palette (dark bg, neon accents like Pinecone)
- [x] Configure typography (clean, modern fonts)
- [x] Set up smooth scroll and page transitions

### Task 1.2: Custom CSS Overhaul (`src/css/custom.css`)
- [x] Define CSS variables for:
  - Primary dark background (#0a0a0a or similar)
  - Accent colors (electric blue, neon green, etc.)
  - Typography scale
  - Spacing system
- [x] Create gradient backgrounds for hero sections
- [x] Add glassmorphism effects for cards
- [x] Implement smooth hover animations

### Task 1.3: Landing Page Components (`src/pages/index.tsx`)
- [x] **Hero Section:** 
  - Animated headline with typing effect
  - Tagline: Something like "Learn Humanoid Robotics Without Losing Your Mind (or Sanity)"
  - CTA buttons: "Start Learning" / "View Curriculum"
  - Subtle robot/AI illustration or animation
  
- [x] **Features Grid (4 Modules Preview):**
  - Module 1: The Robotic Nervous System (ROS 2)
  - Module 2: The Digital Twin (Gazebo & Unity)
  - Module 3: The AI-Robot Brain (NVIDIA Isaac)
  - Module 4: Vision-Language-Action (VLA)
  - Each with icon, title, one-liner, hover effect

- [x] **Course Overview Section:**
  - Timeline/roadmap visualization
  - Estimated time per module
  - Prerequisites (with humor)

- [x] **Testimonials/Social Proof Section:**
  - Fake funny testimonials from "robots" or "AI systems"
  - Example: "Finally, a book that doesn't put me in sleep mode. - GPT-7 (probably)"

- [x] **Footer:**
  - Navigation links
  - Social links
  - Newsletter signup (optional)
  - Copyright with witty comment

### Task 1.4: Homepage Features Component (`src/components/HomepageFeatures/`)
- [x] Redesign `index.tsx` with new feature cards
- [x] Update `styles.module.css` with Pinecone-style animations
- [x] Add hover effects and micro-interactions

---

## 📋 PHASE 2: Documentation Structure & Navigation ✅

### Task 2.1: Sidebar Configuration (`sidebars.ts`)
- [x] Define sidebar structure for 4 modules
- [x] Add custom labels with emojis
- [x] Configure collapsible sections
- [x] Add progress indicators (if possible)

### Task 2.2: Documentation Index (`docs/index.md`)
- [x] Create intro page with:
  - Welcome message (with humor)
  - What you'll learn
  - Who this is for (and who it's NOT for)
  - How to use this book
  - Prerequisites (honest but funny)

### Task 2.3: Module Category Pages
- [x] `docs/module-1-ros2/_category_.json` - Configure Module 1
- [x] `docs/module-2-simulation/_category_.json` - Configure Module 2
- [x] `docs/module-3-isaac/_category_.json` - Configure Module 3
- [x] `docs/module-4-vla/_category_.json` - Configure Module 4

---

## 📋 PHASE 3: Book Content Creation (RAG-Optimized) ✅

### Content Writing Guidelines for RAG:
```
- Each section should be self-contained (can stand alone when retrieved)
- Use clear headers (H2, H3) for easy chunking
- Include code examples with comments
- Keep paragraphs focused on single concepts
- Add "TL;DR" boxes for quick retrieval
- Use consistent terminology throughout
- Include practical examples with context
```

### Task 3.1: Module 1 - The Robotic Nervous System (ROS 2) ✅
- [x] `01-introduction.md` - Why ROS 2? (with rant about ROS 1)
- [x] `02-nodes-topics-services.md` - The holy trinity of robot communication
- [x] `03-python-rclpy.md` - Bridging your Python brain to robot bodies
- [x] `04-urdf-humanoids.md` - Describing your robot's body (like Tinder, but for robots)
- [x] `05-capstone.md` - Mini project with exercises

### Task 3.2: Module 2 - The Digital Twin (Gazebo & Unity) ✅
- [x] `01-introduction.md` - Why simulate when you can break real things?
- [x] `02-gazebo-physics.md` - Physics, gravity, and why your robot keeps falling
- [x] `03-unity-rendering.md` - Making robots look pretty
- [x] `04-sensors-simulation.md` - Fake LiDAR, fake cameras, real problems
- [x] `05-capstone.md` - Build your first virtual disaster zone

### Task 3.3: Module 3 - The AI-Robot Brain (NVIDIA Isaac) ✅
- [x] `01-introduction.md` - NVIDIA: Because gaming GPUs now run robots
- [x] `02-isaac-sim.md` - Photorealistic simulation (for Instagram-ready robots)
- [x] `03-isaac-ros.md` - Hardware-accelerated perception (VSLAM)
- [x] `04-nav2-humanoid.md` - Teaching robots to walk (without falling on their face)
- [x] `05-capstone.md` - Navigation challenge

### Task 3.4: Module 4 - Vision-Language-Action (VLA) ✅
- [x] `01-introduction.md` - When LLMs meet robots (a love story)
- [x] `02-voice-to-action.md` - Whisper sweet nothings to your robot
- [x] `03-cognitive-planning.md` - LLMs as robot brains (what could go wrong?)
- [x] `04-capstone-project.md` - The Autonomous Humanoid (final boss)
- [x] `05-whats-next.md` - Future of Physical AI

---

## 📋 PHASE 4: Interactive Components & Features

### Task 4.1: Custom MDX Components
- [ ] Create `<Callout>` component for tips/warnings/jokes
- [ ] Create `<CodePlayground>` component (optional)
- [ ] Create `<Quiz>` component for end-of-chapter tests
- [ ] Create `<ProgressTracker>` component
- [ ] Create `<TLDRBox>` component for RAG-friendly summaries

### Task 4.2: Animations & Interactions
- [ ] Add scroll-triggered animations
- [ ] Implement code syntax highlighting with custom theme
- [ ] Add copy-to-clipboard for code blocks
- [ ] Create loading animations between pages

---

## 📋 PHASE 5: RAG/Chatbot Preparation

### Task 5.1: Content Optimization for Embedding
- [ ] Ensure each markdown file has proper frontmatter:
  ```yaml
  ---
  id: unique-id
  title: Clear Title
  description: Searchable description
  keywords: [keyword1, keyword2]
  module: module-number
  ---
  ```
- [ ] Add semantic headers throughout content
- [ ] Create `metadata.json` for each module

### Task 5.2: Chunking Strategy Documentation
- [ ] Document chunk size recommendations
- [ ] Define overlap strategy
- [ ] Map content hierarchy for embedding

---

## 📋 PHASE 6: Build & Deploy

### Task 6.1: Build Configuration
- [ ] Optimize `next.config.ts` (if using Next.js features)
- [ ] Configure build output for static hosting
- [ ] Set up proper SEO meta tags

### Task 6.2: Testing
- [ ] Test all internal links
- [ ] Test mobile responsiveness
- [ ] Test dark mode consistency
- [ ] Performance audit (Lighthouse)

### Task 6.3: Deployment
- [ ] Choose hosting (Vercel, Netlify, GitHub Pages)
- [ ] Set up CI/CD pipeline
- [ ] Configure custom domain (if applicable)

---

## 📋 PHASE 7: Future Enhancements (Post-MVP)

- [ ] Integrate RAG chatbot with embedded content
- [ ] Add search functionality (Algolia DocSearch)
- [ ] Add user progress tracking
- [ ] Add community features (comments, discussions)
- [ ] Add video content integration
- [ ] Add interactive robot visualizations

---

## 📁 File Structure After Completion

```
docusaurus-website/
├── docusaurus.config.ts (updated)
├── sidebars.ts (updated)
├── task.md (this file)
├── book-structure.md
├── style-guide.md
├── chunking-strategy.md
├── docs/
│   ├── index.md
│   ├── module-1-ros2/
│   │   ├── _category_.json
│   │   ├── 1.0-intro.md
│   │   ├── 1.1-nodes-topics-services.md
│   │   ├── 1.2-python-agents-rclpy.md
│   │   ├── 1.3-urdf-humanoids.md
│   │   └── 1.4-hands-on-project.md
│   ├── module-2-simulation/
│   │   ├── _category_.json
│   │   ├── 2.0-intro.md
│   │   ├── 2.1-gazebo-physics.md
│   │   ├── 2.2-unity-rendering.md
│   │   ├── 2.3-sensors-simulation.md
│   │   └── 2.4-hands-on-project.md
│   ├── module-3-isaac/
│   │   ├── _category_.json
│   │   ├── 3.0-intro.md
│   │   ├── 3.1-isaac-sim.md
│   │   ├── 3.2-isaac-ros.md
│   │   ├── 3.3-nav2-bipedal.md
│   │   └── 3.4-hands-on-project.md
│   └── module-4-vla/
│       ├── _category_.json
│       ├── 4.0-intro.md
│       ├── 4.1-voice-to-action.md
│       ├── 4.2-cognitive-planning.md
│       ├── 4.3-capstone-project.md
│       └── 4.4-whats-next.md
├── src/
│   ├── components/
│   │   ├── HomepageFeatures/
│   │   │   ├── index.tsx (redesigned)
│   │   │   └── styles.module.css (updated)
│   │   ├── Callout/
│   │   ├── TLDRBox/
│   │   └── Quiz/
│   ├── css/
│   │   └── custom.css (overhauled)
│   └── pages/
│       ├── index.tsx (redesigned landing page)
│       └── index.module.css (updated)
└── static/
    └── img/
        ├── logo.svg
        ├── hero-illustration.svg
        └── module-icons/
```

---

## 🎯 Priority Order

1. **Phase 0** → Clean slate
2. **Phase 1** → Landing page (first impression matters)
3. **Phase 2** → Navigation structure
4. **Phase 3** → Content (the meat)
5. **Phase 4** → Interactive components
6. **Phase 5** → RAG preparation
7. **Phase 6** → Ship it
8. **Phase 7** → Iterate

---

## 💡 Writing Style Notes

### Tone:
- **Sarcastic but educational** - We're teaching serious stuff, but we don't take ourselves too seriously
- **Dark humor** - Acknowledge the pain points of robotics development
- **Adult language** - Not gratuitous, but real (think "damn, this is cool" not "gee whiz")
- **Relatable** - Reference memes, pop culture, developer struggles

### Examples:
- ❌ "In this section, we will learn about ROS 2 nodes."
- ✅ "ROS 2 nodes are like microservices, except when they crash, your $50,000 robot arm crashes into a wall."

- ❌ "URDF files describe robot geometry."
- ✅ "URDF is basically XML that describes your robot's body. Yes, XML. In 2024. I know. We all cope differently."

---

*Last Updated: December 18, 2025*
*Status: Planning Phase*
