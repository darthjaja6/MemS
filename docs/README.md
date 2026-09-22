# MemS project page

A static project overview with method, recorded execution, and implementation details.
No build step or remote dependencies. Project documentation and source live in the repository root, one directory above `docs`.

## Preview

From the repository root:

```bash
python -m http.server 8000 --directory docs
```

Open <http://localhost:8000>.

## GitHub Pages

In the repository's **Settings → Pages**, choose **Deploy from a branch**, select the default branch, and choose the **/docs** folder. Save to publish. All assets use relative paths, so the page works under a project URL such as `https://OWNER.github.io/REPOSITORY/`.

Repository links are inferred from the GitHub Pages address. For a custom domain, set the `spatial-memory:repository` meta tag in `index.html` to the repository URL:

```html
<meta name="spatial-memory:repository" content="https://github.com/OWNER/REPOSITORY">
```

Local previews use this file as the documentation link fallback. JavaScript is used only for repository links and copy buttons; page content and navigation also work without it.

## Recorded figures

`assets/workspace.png` is a screenshot of the local viewer. `assets/stacking.mp4`
is a rendered replay of `examples/stacking/replay.json`, sampled at 10 frames
per second of recorded simulator motion time and encoded at 10 fps. The video
retains the recorded scene updates; it does not add object motion between them.
Both figures include the attributed ARX X5 display model.
