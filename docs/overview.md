# Overview

#### Introduction

When we originally began working on building an autonomous quadcopter we were overwhelmed. There are multiple software systems at varying abstraction levels that blend together into one symphony of software. As a club, we had a relatively simply goal. The computer science division was to produce software that would autonomously takeoff, fly a very short distance, and then land. Unfortunately, we had no idea where to even start. Worse, Panic took so long to design and build that we only had a few months to write the software before our deadline. We poured over research papers, YouTube videos, and other resources to find a direction. A lot of research papers implemented their own quadcopter flight controller software, and as a result, we briefly tried to create our own flight controller by implementing an Estimated Kalman Filter, PID Controllers, and various other pieces of software all on an ATMega 2560 with an off-the-shelf IMU. This proved to be especially difficult with our manpower and lack of general familiarity with such high level concepts. None of our attempts that year ever _took off_, both literally and metaphorically, we just didn't have the expertise or the time. So, we went back to the drawing board and looked at the broader community around DIY drones.

We also decided to reinvent the structure of the computer science division. For the IARC competition, the computer science division is split into multiple sub-sections, each with a specific focus. While members are generally assigned to sub-sections, they are free to collaborate and move between subsections.

* Computer Vision
  * Responsible for visually identifying and cataloguing game components and other objects.
* Simulation

  * Responsible for identifying successful strategies for competing through simulation of the challenge, as well as communicating and coordinating game strategies with Flight Control and Collision Avoidance.

* Flight

  * Responsible for defining and implementing in-air stabilization, general flight trajectory, and any necessary complex maneuvers to perform game strategies as identified by Simulation.

* Collision Avoidance

  * Responsible for defining and implementing techniques to identify obstacles, as well as coordinating with Flight Control to determine maneuvers to avoid obstacles.

* Control

  * Responsible for designing a system to allow communication between the various software subsystems produced by the above groups. Also responsible for writing software for any custom mechanisms such as gimbals or servo controlled arms.

#### ArduPilot

Shortly after we became a design team, we stumbled upon the wonderful world of pre-made flight controllers and open source firmwares for flight controllers. Rather than try and implement our own, we decided to use ArduPilot. It incorporated all of the math and control theory we had previously researched in one package, and is arguably the de-facto standard for autopilots. It is used by both professionals and hobbyists, and most importantly, choosing to use it saves a lot of time. Rather than concern ourselves with problems that had already been solved with proven solutions by ArduPilot and other open source firmwares, we wanted to focus on the more high level challenges of the competition by working on high-level computer vision and obstacle avoidance techniques. We also chose ArduPilot because of its documentation, accessibility, and community. We then paired that firmware with the wonderful open source flight controller hardware project by PX4, the PixHawk. Compared to last year, that represented a dramatic leap in terms of how we were actively attacking our goal. The decision to use ArduPilot also coincided with the introduction of a new mode in the flight software specifically aimed at development in GPS-denied environments. [GUIDED\_NOGPS](http://ardupilot.org/copter/docs/ac2_guidedmode.html#guided-nogps) was officially included in Copter-3.4rc6 and allows companion computers to control the vehicle via attitude commands.

