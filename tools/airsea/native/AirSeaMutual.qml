import QtQuick 2.9
Item {
  width: 1280
  height: 720
  Rectangle { anchors.fill: parent; color: "#14212d" }
  Image { anchors.fill: parent; fillMode: Image.PreserveAspectFit; cache: false; source: "image://mutual/live?" + AirSeaMutual.frameId }
}
