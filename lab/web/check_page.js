// Bullet NEXT Lab - opens the browser page (lab/web/pack_page.py --standalone) on each scene in
// headless Chromium and collects the viewer's "BNL_RESULT scene engine hash same|differs" lines
// (FL, TL.8). Usage: node check_page.js <page.html> <out.json> [scene,scene,...] [timeout_s]
// Needs Playwright with Chromium; WebGL runs on SwiftShader, so frame rates here mean nothing.
const { chromium } = require('playwright');
(async () => {
  const [, , page_path, out, list, tmo] = process.argv;
  const scenes = (list || 'stack_10').split(',');
  const timeout = 1000 * Number(tmo || 240);
  const browser = await chromium.launch({ args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
  const results = {};
  for (const scene of scenes) {
    const page = await browser.newPage({ viewport: { width: 1280, height: 720 } });
    const lines = [];
    page.on('console', m => { const t = m.text(); if (t.startsWith('BNL_RESULT')) lines.push(t); });
    await page.goto('file://' + page_path + '#' + scene);
    const t0 = Date.now();
    while (lines.length < 2 && Date.now() - t0 < timeout) await page.waitForTimeout(1000);
    results[scene] = { lines, seconds: Math.round((Date.now() - t0) / 1000) };
    console.log(scene, JSON.stringify(results[scene]));
    await page.close();
  }
  require('fs').writeFileSync(out, JSON.stringify(results, null, 1));
  await browser.close();
})();
