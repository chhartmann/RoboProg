^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package micro_ros_espidf
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2022-05-25)
------------------
* Update banner (`#143 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/143>`_) (`#144 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/144>`_)
* Fixed build when PATH contains spaces (`#140 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/140>`_) (`#142 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/142>`_)
* Add logos (`#137 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/137>`_) (`#139 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/139>`_)
* Fix service server example (`#128 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/128>`_)
* esp32s3: add esp32s3 support (`#120 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/120>`_)
* Fix atomics (`#121 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/121>`_) (`#123 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/123>`_)
* fix wrong ar command (`#117 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/117>`_)
* Add support for ESP-IDF v4.4 (`#110 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/110>`_) (`#112 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/112>`_)
* custom cmake target: idf.py clean-ros (`#105 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/105>`_)
* Fix embeddedRTPS repo (`#102 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/102>`_)
* Set ESP flag and change rtps branch (`#99 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/99>`_)
* Add embeddedRTPS as experimental middleware (`#95 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/95>`_)
* Fix heartbeat time (`#91 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/91>`_) (`#92 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/92>`_)
* Update dockerfile baseimage (`#87 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/87>`_)
* Add multithread example (`#82 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/82>`_) (`#84 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/84>`_)
* support for vcs import of extra packages (`#83 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/83>`_)
* ping pong application example for esp32 w/idf (`#79 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/79>`_)
* Update README.md
* multichange tool (`#76 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/76>`_)
* Fix utils path (backport `#73 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/73>`_) (`#75 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/75>`_)
* Initial (`#65 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/65>`_)
* Add static type handler (`#68 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/68>`_)
* Enable introspection (`#67 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/67>`_)
* Initial changes for Rolling release
* Update Nightly
* Simplify Colcon installation (`#64 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/64>`_)
* Fix for building after sourcing ROS setup script (`#63 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/63>`_)
* Fix IDF 4.3 atomic 64 exchange (`#62 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/62>`_)
* Update RMW API
* Modify RMW API include (`#41 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/41>`_)
* optional app-colcon.meta in project directory (`#57 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/57>`_)
* Fix example
* Manually merged changes from old branch (`#56 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/56>`_)
* Remove IDF latest from CI (`#55 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/55>`_)
* ESP32c3 support (`#52 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/52>`_)
* Add IDF v4.3 support (`#51 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/51>`_)
* bugfix for ADDITIONAL_CLEAN_FILES (`#49 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/49>`_)
* Fix RCLC foxy (`#48 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/48>`_)
* Fix nightly
* Update custom transport example (`#46 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/46>`_)
* Update rclc repo (`#40 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/40>`_)
* Clean old transports files (`#43 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/43>`_)
* Fix include network util (`#42 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/42>`_)
* Update external transports API (`#38 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/38>`_)
* Fix colcon.meta (`#39 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/39>`_)
* Update nightly with latest IDF (`#37 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/37>`_)
* Add master IDF to CI (`#33 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/33>`_)
* Add issue template
* Add nightly CI
* Support extra packages (`#30 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/30>`_)
* examples: simplifies the examples and pack, when possible, into a single source file.  (`#26 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/26>`_)
* feature/ethernet: Enable support of the microROS over the ethernet interface (`#24 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/24>`_)
* Update README.md
* support for CONFIG_PM_ENABLE added (`#22 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/22>`_)
* Add new micro_ros_msgs repo (`#21 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/21>`_)
* Support for ESP32-S2 (`#19 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/19>`_)
* Rework demo (`#17 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/17>`_)
* rework demo (`#16 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/16>`_)
* make: added -j to enable parallel build jobs (`#15 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/15>`_)
* Update meta defaults (`#14 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/14>`_)
* Add IDF v4.3 support (`#13 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/13>`_)
* Fix rclc spin time
* Update rclc_int32_publisher.c (`#12 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/12>`_)
* Refactor module structure (`#11 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/11>`_)
* Update README.md
* Update README.md
* Enabling Agent autodiscovery
* Update sdkconfig.defaults (`#4 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/4>`_)
* Update README.md
* Removed && in docker run (`#7 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/7>`_)
* Posibility to build in docker container. (`#6 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/6>`_)
* Typo
* Removed libatomic (`#3 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/3>`_)
* Update README.md
* Add Serial support (`#2 <https://github.com/micro-ROS/micro_ros_espidf_component/issues/2>`_)
* Add licensing
* Added CI
* Added agent
* Updated Readme
* Deleted TODO
* Initial commit
* Initial commit
