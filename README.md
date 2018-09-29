# Robot-2018
Code for FRC Team 6705 Wildcat 5e's 2018 robot.
<div class="header">
<div class="subTitle">org.usfirst.frc.team6705.robot</div>
<h2 title="Class DriveTrain" class="title">Class DriveTrain</h2>
</div>
<div class="contentContainer">
<ul class="inheritance">
<li>java.lang.Object</li>
<li>
<ul class="inheritance">
<li>org.usfirst.frc.team6705.robot.DriveTrain</li>
</ul>
</li>
</ul>
<div class="description">
<ul class="blockList">
<li class="blockList">
<hr>
<br>
<pre>public class <span class="typeNameLabel">DriveTrain</span>
extends java.lang.Object</pre>
</li>
</ul>
</div>
<div class="summary">
<ul class="blockList">
<li class="blockList">
<!-- ======== CONSTRUCTOR SUMMARY ======== -->
<ul class="blockList">
<li class="blockList"><a name="constructor.summary">
<!--   -->
</a>
<h3>Constructor Summary</h3>
<table class="memberSummary" border="0" cellpadding="3" cellspacing="0" summary="Constructor Summary table, listing constructors, and an explanation">
<caption><span>Constructors</span><span class="tabEnd">&nbsp;</span></caption>
<tr>
<th class="colOne" scope="col">Constructor and Description</th>
</tr>
<tr class="altColor">
<td class="colOne"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#DriveTrain--">DriveTrain</a></span>()</code>&nbsp;</td>
</tr>
</table>
</li>
</ul>
<!-- ========== METHOD SUMMARY =========== -->
<ul class="blockList">
<li class="blockList"><a name="method.summary">
<!--   -->
</a>
<h3>Method Summary</h3>
<table class="memberSummary" border="0" cellpadding="3" cellspacing="0" summary="Method Summary table, listing methods, and an explanation">
<caption><span id="t0" class="activeTableTab"><span>All Methods</span><span class="tabEnd">&nbsp;</span></span><span id="t1" class="tableTab"><span><a href="javascript:show(1);">Static Methods</a></span><span class="tabEnd">&nbsp;</span></span><span id="t4" class="tableTab"><span><a href="javascript:show(8);">Concrete Methods</a></span><span class="tabEnd">&nbsp;</span></span></caption>
<tr>
<th class="colFirst" scope="col">Modifier and Type</th>
<th class="colLast" scope="col">Method and Description</th>
</tr>
<tr id="i0" class="altColor">
<td class="colFirst"><code>static double</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#getGyro--">getGyro</a></span>()</code>
<div class="block">Get the current angle of the gyro relative to the robot</div>
</td>
</tr>
<tr id="i1" class="rowColor">
<td class="colFirst"><code>static boolean</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#moveByDistance-double-double-">moveByDistance</a></span>(double&nbsp;inches,
              double&nbsp;velocity)</code>
<div class="block">Autonomous moving-moves forward by a specific distance</div>
</td>
</tr>
<tr id="i2" class="altColor">
<td class="colFirst"><code>static boolean</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#moveByDistance-double-double-double-">moveByDistance</a></span>(double&nbsp;inches,
              double&nbsp;heading,
              double&nbsp;velocity)</code>
<div class="block">Autonomous moving-moves forward by a passed in distance with a specific heading (no timeout)</div>
</td>
</tr>
<tr id="i3" class="rowColor">
<td class="colFirst"><code>static boolean</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#moveByDistance-double-double-double-double-">moveByDistance</a></span>(double&nbsp;inches,
              double&nbsp;degrees,
              double&nbsp;velocity,
              double&nbsp;timeOutSeconds)</code>
<div class="block">Autonomous moving-moves forward by passed in distance and turn</div>
</td>
</tr>
<tr id="i4" class="altColor">
<td class="colFirst"><code>static void</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#resetEncoders--">resetEncoders</a></span>()</code>
<div class="block">Reset the encoders back to 0</div>
</td>
</tr>
<tr id="i5" class="rowColor">
<td class="colFirst"><code>static void</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#reverseDriveTrain--">reverseDriveTrain</a></span>()</code>
<div class="block">Reverses all drive train controllers</div>
</td>
</tr>
<tr id="i6" class="altColor">
<td class="colFirst"><code>static boolean</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#runMotionProfile-MotionProfile-">runMotionProfile</a></span>(MotionProfile&nbsp;profile)</code>
<div class="block">Start running the motion profile</div>
</td>
</tr>
<tr id="i7" class="rowColor">
<td class="colFirst"><code>static void</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#setSpeed-double-double-">setSpeed</a></span>(double&nbsp;left,
        double&nbsp;right)</code>
<div class="block">Set the left and right motors to a specific speed (no elevator height adjusting)</div>
</td>
</tr>
<tr id="i8" class="altColor">
<td class="colFirst"><code>static void</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#setup--">setup</a></span>()</code>
<div class="block">Configure all motor controller ports, PID configs, and reset gyro</div>
</td>
</tr>
<tr id="i9" class="rowColor">
<td class="colFirst"><code>static void</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#setupMotionProfile-MotionProfile-">setupMotionProfile</a></span>(MotionProfile&nbsp;profile)</code>
<div class="block">Set up a motion profile</div>
</td>
</tr>
<tr id="i10" class="altColor">
<td class="colFirst"><code>static void</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#setVelocity-double-double-">setVelocity</a></span>(double&nbsp;left,
           double&nbsp;right)</code>
<div class="block">Set the velocity of the left and right motors (slows down proportionally to elevator height)</div>
</td>
</tr>
<tr id="i11" class="rowColor">
<td class="colFirst"><code>static void</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#startMotionProfile-MotionProfile-">startMotionProfile</a></span>(MotionProfile&nbsp;profile)</code>
<div class="block">Begin the motion profile</div>
</td>
</tr>
<tr id="i12" class="altColor">
<td class="colFirst"><code>static void</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#stop--">stop</a></span>()</code>
<div class="block">Stop both motors</div>
</td>
</tr>
<tr id="i13" class="rowColor">
<td class="colFirst"><code>static void</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#switchToMotionProfile--">switchToMotionProfile</a></span>()</code>
<div class="block">Switch control mode to motion profiling</div>
</td>
</tr>
<tr id="i14" class="altColor">
<td class="colFirst"><code>static void</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#switchToVelocityMode--">switchToVelocityMode</a></span>()</code>
<div class="block">Switch control mode to velocity mode</div>
</td>
</tr>
<tr id="i15" class="rowColor">
<td class="colFirst"><code>static void</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#tankDrive-double-double-">tankDrive</a></span>(double&nbsp;leftSpeed,
         double&nbsp;rightSpeed)</code>
<div class="block">Teleop control tank drive, sets left and right motors to passed in speeds</div>
</td>
</tr>
<tr id="i16" class="altColor">
<td class="colFirst"><code>static boolean</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#turnDegrees-double-">turnDegrees</a></span>(double&nbsp;degrees)</code>
<div class="block">Autonomous turning method</div>
</td>
</tr>
<tr id="i17" class="rowColor">
<td class="colFirst"><code>static boolean</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#turnDegrees-double-double-">turnDegrees</a></span>(double&nbsp;degrees,
           double&nbsp;timeOutDegreeTolerance)</code>
<div class="block">Autonomous turning method</div>
</td>
</tr>
<tr id="i18" class="altColor">
<td class="colFirst"><code>static void</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#undoReverseDriveTrain--">undoReverseDriveTrain</a></span>()</code>
<div class="block">Reset drive train directions back to default</div>
</td>
</tr>
<tr id="i19" class="rowColor">
<td class="colFirst"><code>static boolean</code></td>
<td class="colLast"><code><span class="memberNameLink"><a href="../../../../../org/usfirst/frc/team6705/robot/DriveTrain.html#wait-double-double-">wait</a></span>(double&nbsp;time,
    double&nbsp;previousTime)</code>
<div class="block">Wait for a specific amount of time</div>
</td>
</tr>
</table>
<ul class="blockList">
<li class="blockList"><a name="methods.inherited.from.class.java.lang.Object">
<!--   -->
</a>
<h3>Methods inherited from class&nbsp;java.lang.Object</h3>
<code>clone, equals, finalize, getClass, hashCode, notify, notifyAll, toString, wait, wait, wait</code></li>
</ul>
</li>
</ul>
</li>
</ul>
</div>
<div class="details">
<ul class="blockList">
<li class="blockList">
<!-- ========= CONSTRUCTOR DETAIL ======== -->
<ul class="blockList">
<li class="blockList"><a name="constructor.detail">
<!--   -->
</a>
<h3>Constructor Detail</h3>
<a name="DriveTrain--">
<!--   -->
</a>
<ul class="blockListLast">
<li class="blockList">
<h4>DriveTrain</h4>
<pre>public&nbsp;DriveTrain()</pre>
</li>
</ul>
</li>
</ul>
<!-- ============ METHOD DETAIL ========== -->
<ul class="blockList">
<li class="blockList"><a name="method.detail">
<!--   -->
</a>
<h3>Method Detail</h3>
<a name="setup--">
<!--   -->
</a>
<ul class="blockList">
<li class="blockList">
<h4>setup</h4>
<pre>public static&nbsp;void&nbsp;setup()</pre>
<div class="block">Configure all motor controller ports, PID configs, and reset gyro</div>
</li>
</ul>
<a name="reverseDriveTrain--">
<!--   -->
</a>
<ul class="blockList">
<li class="blockList">
<h4>reverseDriveTrain</h4>
<pre>public static&nbsp;void&nbsp;reverseDriveTrain()</pre>
<div class="block">Reverses all drive train controllers</div>
</li>
</ul>
<a name="undoReverseDriveTrain--">
<!--   -->
</a>
<ul class="blockList">
<li class="blockList">
<h4>undoReverseDriveTrain</h4>
<pre>public static&nbsp;void&nbsp;undoReverseDriveTrain()</pre>
<div class="block">Reset drive train directions back to default</div>
</li>
</ul>
<a name="switchToVelocityMode--">
<!--   -->
</a>
<ul class="blockList">
<li class="blockList">
<h4>switchToVelocityMode</h4>
<pre>public static&nbsp;void&nbsp;switchToVelocityMode()</pre>
<div class="block">Switch control mode to velocity mode</div>
</li>
</ul>
<a name="switchToMotionProfile--">
<!--   -->
</a>
<ul class="blockList">
<li class="blockList">
<h4>switchToMotionProfile</h4>
<pre>public static&nbsp;void&nbsp;switchToMotionProfile()</pre>
<div class="block">Switch control mode to motion profiling</div>
</li>
</ul>
<a name="tankDrive-double-double-">
<!--   -->
</a>
<ul class="blockList">
<li class="blockList">
<h4>tankDrive</h4>
<pre>public static&nbsp;void&nbsp;tankDrive(double&nbsp;leftSpeed,
                             double&nbsp;rightSpeed)</pre>
<div class="block">Teleop control tank drive, sets left and right motors to passed in speeds</div>
<dl>
<dt><span class="paramLabel">Parameters:</span></dt>
<dd><code>leftSpeed</code> - speed to set the left wheels to</dd>
<dd><code>rightSpeed</code> - speed to set the right wheels to</dd>
</dl>
</li>
</ul>
<a name="moveByDistance-double-double-double-double-">
<!--   -->
</a>
<ul class="blockList">
<li class="blockList">
<h4>moveByDistance</h4>
<pre>public static&nbsp;boolean&nbsp;moveByDistance(double&nbsp;inches,
                                     double&nbsp;degrees,
                                     double&nbsp;velocity,
                                     double&nbsp;timeOutSeconds)</pre>
<div class="block">Autonomous moving-moves forward by passed in distance and turn</div>
<dl>
<dt><span class="paramLabel">Parameters:</span></dt>
<dd><code>inches</code> - distance to move in inches</dd>
<dd><code>degrees</code> - amount to turn in degrees (positive is counterclockwise)</dd>
<dd><code>velocity</code> - velocity to move at</dd>
<dd><code>timeOutSeconds</code> - amount of time to stop after if the action is not completed</dd>
<dt><span class="returnLabel">Returns:</span></dt>
<dd>true when the action is complete</dd>
</dl>
</li>
</ul>
<a name="moveByDistance-double-double-double-">
<!--   -->
</a>
<ul class="blockList">
<li class="blockList">
<h4>moveByDistance</h4>
<pre>public static&nbsp;boolean&nbsp;moveByDistance(double&nbsp;inches,
                                     double&nbsp;heading,
                                     double&nbsp;velocity)</pre>
<div class="block">Autonomous moving-moves forward by a passed in distance with a specific heading (no timeout)</div>
<dl>
<dt><span class="paramLabel">Parameters:</span></dt>
<dd><code>inches</code> - distance to move by</dd>
<dd><code>heading</code> - angle to turn at</dd>
<dd><code>velocity</code> - velocity to move at</dd>
<dt><span class="returnLabel">Returns:</span></dt>
<dd>true when the action is complete</dd>
</dl>
</li>
</ul>
<a name="moveByDistance-double-double-">
<!--   -->
</a>
<ul class="blockList">
<li class="blockList">
<h4>moveByDistance</h4>
<pre>public static&nbsp;boolean&nbsp;moveByDistance(double&nbsp;inches,
                                     double&nbsp;velocity)</pre>
<div class="block">Autonomous moving-moves forward by a specific distance</div>
<dl>
<dt><span class="paramLabel">Parameters:</span></dt>
<dd><code>inches</code> - distance to move by</dd>
<dd><code>velocity</code> - velocity to move at</dd>
<dt><span class="returnLabel">Returns:</span></dt>
<dd>true when the action is complete</dd>
</dl>
</li>
</ul>
<a name="turnDegrees-double-double-">
<!--   -->
</a>
<ul class="blockList">
<li class="blockList">
<h4>turnDegrees</h4>
<pre>public static&nbsp;boolean&nbsp;turnDegrees(double&nbsp;degrees,
                                  double&nbsp;timeOutDegreeTolerance)</pre>
<div class="block">Autonomous turning method</div>
<dl>
<dt><span class="paramLabel">Parameters:</span></dt>
<dd><code>degrees</code> - amount to turn by</dd>
<dd><code>timeOutDegreeTolerance</code> - tolerance for acceptable turning variation</dd>
<dt><span class="returnLabel">Returns:</span></dt>
<dd>true when the action is complete</dd>
</dl>
</li>
</ul>
<a name="turnDegrees-double-">
<!--   -->
</a>
<ul class="blockList">
<li class="blockList">
<h4>turnDegrees</h4>
<pre>public static&nbsp;boolean&nbsp;turnDegrees(double&nbsp;degrees)</pre>
<div class="block">Autonomous turning method</div>
<dl>
<dt><span class="paramLabel">Parameters:</span></dt>
<dd><code>degrees</code> - amount to turn by</dd>
<dt><span class="returnLabel">Returns:</span></dt>
<dd>true when the action is complete</dd>
</dl>
</li>
</ul>
<a name="setupMotionProfile-MotionProfile-">
<!--   -->
</a>
<ul class="blockList">
<li class="blockList">
<h4>setupMotionProfile</h4>
<pre>public static&nbsp;void&nbsp;setupMotionProfile(MotionProfile&nbsp;profile)</pre>
<div class="block">Set up a motion profile</div>
<dl>
<dt><span class="paramLabel">Parameters:</span></dt>
<dd><code>profile</code> - the profile to set up</dd>
</dl>
</li>
</ul>
<a name="startMotionProfile-MotionProfile-">
<!--   -->
</a>
<ul class="blockList">
<li class="blockList">
<h4>startMotionProfile</h4>
<pre>public static&nbsp;void&nbsp;startMotionProfile(MotionProfile&nbsp;profile)</pre>
<div class="block">Begin the motion profile</div>
<dl>
<dt><span class="paramLabel">Parameters:</span></dt>
<dd><code>profile</code> - the profile to start</dd>
</dl>
</li>
</ul>
<a name="runMotionProfile-MotionProfile-">
<!--   -->
</a>
<ul class="blockList">
<li class="blockList">
<h4>runMotionProfile</h4>
<pre>public static&nbsp;boolean&nbsp;runMotionProfile(MotionProfile&nbsp;profile)</pre>
<div class="block">Start running the motion profile</div>
<dl>
<dt><span class="paramLabel">Parameters:</span></dt>
<dd><code>profile</code> - the profile to run</dd>
<dt><span class="returnLabel">Returns:</span></dt>
<dd>true when the profile is finished running</dd>
</dl>
</li>
</ul>
<a name="setVelocity-double-double-">
<!--   -->
</a>
<ul class="blockList">
<li class="blockList">
<h4>setVelocity</h4>
<pre>public static&nbsp;void&nbsp;setVelocity(double&nbsp;left,
                               double&nbsp;right)</pre>
<div class="block">Set the velocity of the left and right motors (slows down proportionally to elevator height)</div>
<dl>
<dt><span class="paramLabel">Parameters:</span></dt>
<dd><code>left</code> - left motor speed to set</dd>
<dd><code>right</code> - right motor speed to set</dd>
</dl>
</li>
</ul>
<a name="setSpeed-double-double-">
<!--   -->
</a>
<ul class="blockList">
<li class="blockList">
<h4>setSpeed</h4>
<pre>public static&nbsp;void&nbsp;setSpeed(double&nbsp;left,
                            double&nbsp;right)</pre>
<div class="block">Set the left and right motors to a specific speed (no elevator height adjusting)</div>
<dl>
<dt><span class="paramLabel">Parameters:</span></dt>
<dd><code>left</code> - left motor speed to set</dd>
<dd><code>right</code> - right motor speed to set</dd>
</dl>
</li>
</ul>
<a name="stop--">
<!--   -->
</a>
<ul class="blockList">
<li class="blockList">
<h4>stop</h4>
<pre>public static&nbsp;void&nbsp;stop()</pre>
<div class="block">Stop both motors</div>
</li>
</ul>
<a name="wait-double-double-">
<!--   -->
</a>
<ul class="blockList">
<li class="blockList">
<h4>wait</h4>
<pre>public static&nbsp;boolean&nbsp;wait(double&nbsp;time,
                           double&nbsp;previousTime)</pre>
<div class="block">Wait for a specific amount of time</div>
<dl>
<dt><span class="paramLabel">Parameters:</span></dt>
<dd><code>time</code> - the time to wait</dd>
<dd><code>previousTime</code> - the starting time when the command was called</dd>
<dt><span class="returnLabel">Returns:</span></dt>
<dd>true when the wait is over</dd>
</dl>
</li>
</ul>
<a name="getGyro--">
<!--   -->
</a>
<ul class="blockList">
<li class="blockList">
<h4>getGyro</h4>
<pre>public static&nbsp;double&nbsp;getGyro()</pre>
<div class="block">Get the current angle of the gyro relative to the robot</div>
<dl>
<dt><span class="returnLabel">Returns:</span></dt>
<dd>-1 * the current gyro angle</dd>
</dl>
</li>
</ul>
<a name="resetEncoders--">
<!--   -->
</a>
<ul class="blockListLast">
<li class="blockList">
<h4>resetEncoders</h4>
<pre>public static&nbsp;void&nbsp;resetEncoders()</pre>
<div class="block">Reset the encoders back to 0</div>
</li>
</ul>
</li>
</ul>
</li>
</ul>
</div>
</div>
