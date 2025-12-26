function setDrawioDarkMode(enable) {
  let elements = document.querySelectorAll(".mxgraph");
  elements.forEach((element) => {
    let data = JSON.parse(element.dataset.mxgraph);
    data["dark-mode"] = enable ? "dark" : "light";
    element.dataset.mxgraph = JSON.stringify(data);
  });
}

function isPageDark() {
  return __md_get("__palette").index === 1;
}

function reloadGraph() {
  console.debug("Reloading graph");
  const has_graph_viewer = typeof GraphViewer !== "undefined";
  (has_graph_viewer && GraphViewer.processElements()) ||
    console.debug("GraphViewer not yet loaded");
}

document$.subscribe(() => {
  setDrawioDarkMode(isPageDark());
  reloadGraph();
});

document.getElementById("__palette_0").addEventListener("change", () => {
  console.log("Switched to light mode");
  setDrawioDarkMode(false);
  reloadGraph();
});

document.getElementById("__palette_1").addEventListener("change", () => {
  console.log("Switched to dark mode");
  setDrawioDarkMode(true);
  reloadGraph();
});
