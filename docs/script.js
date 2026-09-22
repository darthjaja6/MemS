"use strict";

// Relative assets work both at a domain root and under a GitHub Pages project path.
// Set this meta tag to the repository URL when using a custom domain.
const configuredRepository = document.querySelector('meta[name="spatial-memory:repository"]').content.trim();
let repository = configuredRepository;
if (!repository && location.hostname.endsWith(".github.io")) {
  const owner = location.hostname.slice(0, -".github.io".length);
  const project = location.pathname.split("/").filter(Boolean)[0];
  repository = `https://github.com/${owner}/${project || `${owner}.github.io`}`;
}
if (/^https:\/\/github\.com\/[^/]+\/[^/]+\/?$/.test(repository)) {
  document.querySelectorAll("[data-source]").forEach(link => {
    link.href = repository.replace(/\/$/, "") + (link.dataset.path ? "/blob/HEAD/" + link.dataset.path : "");
  });
}

document.querySelectorAll("[data-copy]").forEach(button => {
  button.addEventListener("click", async () => {
    const code = document.getElementById(button.dataset.copy);
    const announcement = document.querySelector(".copy-announcement");
    try {
      await navigator.clipboard.writeText(code.textContent);
      button.textContent = "Copied";
      announcement.textContent = "Commands copied to clipboard.";
      setTimeout(() => { button.textContent = "Copy"; }, 2000);
    } catch {
      const range = document.createRange();
      range.selectNodeContents(code);
      const selection = window.getSelection();
      selection.removeAllRanges();
      selection.addRange(range);
      announcement.textContent = "Commands selected. Use your browser's copy command.";
    }
  });
});
