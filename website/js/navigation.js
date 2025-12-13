/**
 * Navigation and Smooth Scrolling
 * Handles smooth scrolling, TOC highlighting, and back-to-top button
 */

(function() {
  'use strict';

  // Smooth scroll to anchor links
  function initSmoothScrolling() {
    document.querySelectorAll('a[href^="#"]').forEach(anchor => {
      anchor.addEventListener('click', function (e) {
        const href = this.getAttribute('href');

        // Ignore empty hash links
        if (href === '#' || href === '#!') {
          return;
        }

        const target = document.querySelector(href);
        if (target) {
          e.preventDefault();

          // Calculate offset for sticky navbar
          const navbarHeight = document.querySelector('.navbar')?.offsetHeight || 0;
          const targetPosition = target.getBoundingClientRect().top + window.pageYOffset - navbarHeight - 20;

          window.scrollTo({
            top: targetPosition,
            behavior: 'smooth'
          });

          // Update URL without jumping
          if (history.pushState) {
            history.pushState(null, null, href);
          }
        }
      });
    });
  }

  // Highlight active section in TOC
  function initTOCHighlighting() {
    const toc = document.querySelector('.toc');
    if (!toc) return;

    const sections = document.querySelectorAll('section[id], h2[id], h3[id]');
    const navLinks = document.querySelectorAll('.toc a');

    if (sections.length === 0 || navLinks.length === 0) return;

    function highlightActiveSection() {
      let current = '';
      const navbarHeight = document.querySelector('.navbar')?.offsetHeight || 0;

      sections.forEach(section => {
        const sectionTop = section.offsetTop;
        const sectionHeight = section.offsetHeight;

        if (window.pageYOffset >= sectionTop - navbarHeight - 100) {
          current = section.getAttribute('id');
        }
      });

      navLinks.forEach(link => {
        link.classList.remove('active');
        const href = link.getAttribute('href');
        if (href === `#${current}`) {
          link.classList.add('active');
        }
      });
    }

    // Throttle scroll event
    let ticking = false;
    window.addEventListener('scroll', function() {
      if (!ticking) {
        window.requestAnimationFrame(function() {
          highlightActiveSection();
          ticking = false;
        });
        ticking = true;
      }
    });

    // Initial highlight
    highlightActiveSection();
  }

  // Back to top button
  function initBackToTop() {
    const backToTop = document.querySelector('.back-to-top');
    if (!backToTop) return;

    function toggleBackToTop() {
      if (window.pageYOffset > 300) {
        backToTop.classList.add('visible');
      } else {
        backToTop.classList.remove('visible');
      }
    }

    // Throttle scroll event
    let ticking = false;
    window.addEventListener('scroll', function() {
      if (!ticking) {
        window.requestAnimationFrame(function() {
          toggleBackToTop();
          ticking = false;
        });
        ticking = true;
      }
    });

    // Initial check
    toggleBackToTop();
  }

  // Highlight current page in navbar
  function initNavbarHighlighting() {
    const currentPage = window.location.pathname.split('/').pop() || 'index.html';
    const navLinks = document.querySelectorAll('.nav-links a');

    navLinks.forEach(link => {
      const linkPage = link.getAttribute('href');
      if (linkPage === currentPage ||
          (currentPage === '' && linkPage === 'index.html') ||
          (currentPage === 'index.html' && linkPage === './')) {
        link.classList.add('active');
      }
    });
  }

  // External links: Open in new tab
  function initExternalLinks() {
    document.querySelectorAll('a[href^="http"]').forEach(link => {
      if (!link.hostname.includes(window.location.hostname)) {
        link.setAttribute('target', '_blank');
        link.setAttribute('rel', 'noopener noreferrer');
      }
    });
  }

  // Copy code blocks to clipboard
  function initCodeCopy() {
    document.querySelectorAll('pre code').forEach(codeBlock => {
      const pre = codeBlock.parentElement;
      const button = document.createElement('button');
      button.className = 'copy-code-btn';
      button.textContent = 'Copy';
      button.style.cssText = `
        position: absolute;
        top: 0.5em;
        right: 0.5em;
        padding: 0.3em 0.6em;
        font-size: 0.8em;
        background: var(--accent-blue);
        color: white;
        border: none;
        border-radius: 3px;
        cursor: pointer;
        opacity: 0;
        transition: opacity 0.2s ease;
      `;

      pre.style.position = 'relative';
      pre.appendChild(button);

      pre.addEventListener('mouseenter', () => {
        button.style.opacity = '1';
      });

      pre.addEventListener('mouseleave', () => {
        button.style.opacity = '0';
      });

      button.addEventListener('click', () => {
        const code = codeBlock.textContent;
        navigator.clipboard.writeText(code).then(() => {
          button.textContent = 'Copied!';
          setTimeout(() => {
            button.textContent = 'Copy';
          }, 2000);
        }).catch(err => {
          console.error('Failed to copy:', err);
          button.textContent = 'Failed';
          setTimeout(() => {
            button.textContent = 'Copy';
          }, 2000);
        });
      });
    });
  }

  // Initialize all features when DOM is ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }

  function init() {
    initSmoothScrolling();
    initTOCHighlighting();
    initBackToTop();
    initNavbarHighlighting();
    initExternalLinks();
    initCodeCopy();
  }
})();
