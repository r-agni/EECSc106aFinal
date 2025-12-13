# Drone Cooperative System - Project Website

Academic website for UC Berkeley EECS/ME 106a Final Project.

## Team 14
- Agni Rajinikanth
- Alan Bao
- Nohl Abdelhadi
- Imam Majed Alayeh

## Project Title
**Drone Cooperative System for Humanitarian Missions**

## Website Structure

```
website/
├── index.html              # Home page
├── technical.html          # Technical details page
├── css/
│   ├── paper.css          # LaTeX-inspired typography
│   ├── main.css           # Layout and components
│   └── responsive.css     # Mobile breakpoints
├── js/
│   └── navigation.js      # Smooth scrolling and TOC
├── assets/
│   └── images/            # Project images
└── README.md
```

## Local Testing

To test the website locally:

```bash
cd website
python -m http.server 8000
```

Then visit: http://localhost:8000

## GitHub Pages Deployment

This website is configured to be deployed via GitHub Pages.

### Deployment Steps:

1. Ensure all files are committed to the repository
2. Go to repository Settings → Pages
3. Set Source to: **Deploy from a branch**
4. Select branch: `main`
5. Select folder: `/website`
6. Click Save

The website will be available at:
**https://nohlaaaron.github.io/EECSc106aFinal/**

## Features

- **Academic Paper Styling:** LaTeX-inspired typography using Computer Modern font
- **Responsive Design:** Mobile-friendly layout with breakpoints for tablet and phone
- **MathJax Integration:** Beautiful equation rendering with LaTeX syntax
- **Smooth Navigation:** Animated scrolling and active section highlighting
- **Video Embeds:** YouTube integration for demo videos
- **Professional Layout:** Berkeley Blue and California Gold color scheme

## Technologies Used

- Plain HTML5, CSS3, JavaScript (no frameworks)
- MathJax 3 for equation rendering
- Latin Modern Roman font (Computer Modern equivalent)
- Responsive CSS Grid and Flexbox

## Content

### Home Page (index.html)
- Project title and team information
- Abstract and system overview
- Demo video and results
- Challenges and solutions
- Conclusion and team photo

### Technical Details Page (technical.html)
- AR Drone algorithms (ArUco, grid search, transforms)
- Tello Drone algorithms (YOLOv8, PID, dead reckoning, Bresenham)
- Mathematical formulations and equations
- Implementation details and code references
- System integration architecture

## Video Links

- **Final Demo:** https://youtu.be/ZG6kzSA0JAc
- **Drone Centering:** https://youtu.be/NWBAYWgyoZQ
- **Self Centering:** https://youtube.com/shorts/0HOplPK0Apg

## GitHub Repository

https://github.com/NohlAaron/EECSc106aFinal

## License

© 2025 Team 14 | UC Berkeley EECS/ME 106a
