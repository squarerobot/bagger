^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bagger
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.5 (2024-01-02)
-----------
* Downgrade the ROS message severity of all warnings having to do with
  recording states not changing due to them already being in the 
  requested state
* Contributors: Brenden Gibbons

0.1.4 (2021-04-26)
-----------
* Proper waiting for record processing. Fixes `#6 <https://github.com/squarerobot/bagger/issues/6>`_
* Fix test timing synchronisity issue.
  Tests were beginning before the last test's latest bag_states message
  was published.
* Contributors: Brenden Gibbons

0.1.3 (2018-10-04)
------------------
* Infer and publish bag names and directories for each bag profile
* Add CL option to not decompress bags.  Fixes `#4 <https://github.com/squarerobot/bagger/issues/4>`_
* Contributors: Brenden Gibbons

0.1.2 (2018-07-12)
------------------
* Fix some urls and Fix `#1 <https://github.com/squarerobot/bagger/issues/1>`_
* Contributors: Brenden Gibbons

0.1.1 (2018-06-29)
------------------
* Initial Commit
* Contributors: Brenden Gibbons
