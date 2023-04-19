# vclip

https://github.com/fkanehiro/hrpsys-base/tree/master/rtc/CollisionDetector/vclip_1.0 で公開されているソースコードに、以下の改変を加えたもの

- qhullの代わりにqhull_rを使用 (参考 http://www.qhull.org/html/qh-code.htm https://github.com/PointCloudLibrary/pcl/pull/4540).
  - qhull_rは、qhullをマルチスレッド対応したもの
  - qhullとqhull_rは関数名が同じなので、qhull_rに依存したライブラリとqhullに依存したライブラリが混在していると実行時にエラーになる場合があることに注意

- vclipをマルチスレッドに対応
  - static変数を、非static変数に変えた
