<mxfile host="Electron" modified="2025-04-07T08:38:59.298Z" agent="Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) draw.io/22.1.2 Chrome/114.0.5735.289 Electron/25.9.4 Safari/537.36" etag="f53VCbJUrk_HXLackEPv" version="22.1.2" type="device">
  <diagram name="Page-1" id="S2h_ZQmrx3T9z0P8xpj5">
    <mxGraphModel dx="2526" dy="1609" grid="1" gridSize="10" guides="1" tooltips="1" connect="1" arrows="1" fold="1" page="1" pageScale="1" pageWidth="827" pageHeight="1169" math="0" shadow="0">
      <root>
        <mxCell id="0" />
        <mxCell id="1" parent="0" />
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-1" value="SerialLinkBase" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=22.794117647058822;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="153" y="201" width="1194" height="789.7941176470588" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-2" value="-bool _redundantTaskSet" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="22.794117647058822" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-3" value="-double _jointPositionGain" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="45.794117647058826" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-4" value="-double _jointVelocityGain" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="68.79411764705883" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-5" value="-double _manipulability" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="91.79411764705883" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-6" value="-double _minManipulability" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="114.79411764705883" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-7" value="-double _maxJointAcceleration" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="137.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-8" value="-double _controlFrequency" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="160.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-9" value="-Eigen::Matrix _cartesianStiffness" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="183.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-10" value="-Eigen::Matrix _cartesianDamping" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="206.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-11" value="-Eigen::Matrix _jacobianMatrix" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="229.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-12" value="-Eigen::Matrix _forceEllipsoid" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="252.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-13" value="-Eigen::Matrix _constraintMatrix" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="275.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-14" value="-Eigen::VectorXd _constraintVector" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="298.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-15" value="-Eigen::VectorXd _redundantTask" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="321.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-16" value="-std::shared_ptr _model" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="344.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-17" value="-Pose _endpointPose" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="367.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-18" value="-ReferenceFrame *_endpointFrame" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="390.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-19" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="413.79411764705884" width="1194" height="8" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-20" value="+SerialLinkBase(std::shared_ptr model, const std::string &amp;amp;endpointName, const Parameters ¶meters)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="421.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-21" value="+Eigen::VectorXd resolve_endpoint_motion(const Eigen::Vector &amp;amp;endpointMotion)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="444.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-22" value="+Eigen::VectorXd resolve_endpoint_twist(const Eigen::Vector &amp;amp;twist)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="467.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-23" value="+Eigen::VectorXd track_endpoint_trajectory(const Pose &amp;amp;desiredPose, const Eigen::Vector &amp;amp;desiredVelocity, const Eigen::Vector &amp;amp;desiredAcceleration)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="490.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-24" value="+Eigen::VectorXd track_joint_trajectory(const Eigen::VectorXd &amp;desiredPosition, const Eigen::VectorXd &amp;desiredVelocity, const Eigen::VectorXd &amp;desiredAcceleration)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="513.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-25" value="+double manipulability() const" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="536.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-26" value="+Eigen::VectorXd manipulability_gradient()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="559.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-27" value="+Pose endpoint_pose() const" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="582.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-28" value="+Eigen::Vector endpoint_velocity() const" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="605.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-29" value="+Eigen::Matrix jacobian() const" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="628.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-30" value="+void update()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="651.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-31" value="+void set_control_parameters(const Parameters &amp;parameters)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="674.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-32" value="+bool set_redundant_task(const Eigen::VectorXd &amp;task)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="697.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-33" value="+bool is_singular()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="720.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-34" value="+std::shared_ptr model() const" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="743.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-35" value="+double frequency() const" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry y="766.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-36" value="&lt;&lt;abstract&gt;&gt;&#xa;QPSolver" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=46.5;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="20" y="50" width="106" height="85.5" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-37" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-36">
          <mxGeometry y="46.5" width="106" height="8" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-38" value="+void solve()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-36">
          <mxGeometry y="54.5" width="106" height="31" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-39" value="KinematicTree" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=28.4;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="176" y="61" width="258" height="64.4" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-40" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-39">
          <mxGeometry y="28.4" width="258" height="8" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-41" value="+Eigen::VectorXd joint_velocities()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-39">
          <mxGeometry y="36.4" width="258" height="28" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-42" value="Pose" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=26.571428571428573;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="484" y="50" width="225" height="88.57142857142857" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-43" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-42">
          <mxGeometry y="26.571428571428573" width="225" height="8" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-44" value="+Eigen::VectorXd position()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-42">
          <mxGeometry y="34.57142857142857" width="225" height="27" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-45" value="+Eigen::Matrix3d orientation()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-42">
          <mxGeometry y="61.57142857142857" width="225" height="27" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-46" value="ReferenceFrame" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=28.4;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="760" y="61" width="288" height="64.4" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-47" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-46">
          <mxGeometry y="28.4" width="288" height="8" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-48" value="+Eigen::Matrix transformation_matrix()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-46">
          <mxGeometry y="36.4" width="288" height="28" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-49" value="Limits" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=26.571428571428573;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="1097" y="50" width="155" height="93.57142857142857" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-50" value="+double lower_limit" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-49">
          <mxGeometry y="26.571428571428573" width="155" height="27" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-51" value="+double upper_limit" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-49">
          <mxGeometry y="53.57142857142857" width="155" height="27" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-52" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-49">
          <mxGeometry y="80.57142857142857" width="155" height="13" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-53" value="Parameters" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=26.571428571428573;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="1302" y="50" width="159" height="93.57142857142857" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-54" value="+double controlGain" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-53">
          <mxGeometry y="26.571428571428573" width="159" height="27" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-55" value="+double frequency" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-53">
          <mxGeometry y="53.57142857142857" width="159" height="27" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-56" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-53">
          <mxGeometry y="80.57142857142857" width="159" height="13" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-57" value="" style="curved=1;startArrow=block;startSize=16;startFill=0;endArrow=none;exitX=0.5007001768867925;exitY=1;entryX=0.000032715661641541036;entryY=0.030882908209860917;rounded=0;" edge="1" parent="1" source="cwrhPxI-1V7dXrqJGJlZ-36" target="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="73" y="176" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-58" value="" style="curved=1;startArrow=block;startSize=16;startFill=0;endArrow=none;exitX=0.5010446947674418;exitY=1;entryX=0.14989967799305653;entryY=0;rounded=0;" edge="1" parent="1" source="cwrhPxI-1V7dXrqJGJlZ-39" target="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="305" y="176" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-59" value="" style="curved=1;startArrow=block;startSize=16;startFill=0;endArrow=none;exitX=0.5024652777777778;exitY=1;entryX=0.37960741771583;entryY=0;rounded=0;" edge="1" parent="1" source="cwrhPxI-1V7dXrqJGJlZ-42" target="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="597" y="176" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-60" value="" style="curved=1;startArrow=block;startSize=16;startFill=0;endArrow=none;exitX=0.4982096354166667;exitY=1;entryX=0.6208440584148233;entryY=0;rounded=0;" edge="1" parent="1" source="cwrhPxI-1V7dXrqJGJlZ-46" target="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="903" y="176" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-61" value="" style="curved=1;startArrow=block;startSize=16;startFill=0;endArrow=none;exitX=0.5020917338709677;exitY=1;entryX=0.8344562345981106;entryY=0;rounded=0;" edge="1" parent="1" source="cwrhPxI-1V7dXrqJGJlZ-49" target="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="1175" y="176" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-62" value="" style="curved=1;startArrow=block;startSize=16;startFill=0;endArrow=none;exitX=0.5014249213836478;exitY=1;entryX=0.9973400164085734;entryY=0;rounded=0;" edge="1" parent="1" source="cwrhPxI-1V7dXrqJGJlZ-53" target="cwrhPxI-1V7dXrqJGJlZ-1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="1382" y="176" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-63" value="SerialLinkBase" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=22.794117647058822;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="153" y="201" width="1194" height="789.7941176470588" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-64" value="-bool _redundantTaskSet" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="22.794117647058822" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-65" value="-double _jointPositionGain" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="45.794117647058826" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-66" value="-double _jointVelocityGain" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="68.79411764705883" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-67" value="-double _manipulability" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="91.79411764705883" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-68" value="-double _minManipulability" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="114.79411764705883" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-69" value="-double _maxJointAcceleration" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="137.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-70" value="-double _controlFrequency" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="160.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-71" value="-Eigen::Matrix _cartesianStiffness" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="183.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-72" value="-Eigen::Matrix _cartesianDamping" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="206.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-73" value="-Eigen::Matrix _jacobianMatrix" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="229.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-74" value="-Eigen::Matrix _forceEllipsoid" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="252.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-75" value="-Eigen::Matrix _constraintMatrix" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="275.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-76" value="-Eigen::VectorXd _constraintVector" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="298.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-77" value="-Eigen::VectorXd _redundantTask" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="321.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-78" value="-std::shared_ptr _model" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="344.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-79" value="-Pose _endpointPose" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="367.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-80" value="-ReferenceFrame *_endpointFrame" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="390.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-81" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="413.79411764705884" width="1194" height="8" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-82" value="+SerialLinkBase(std::shared_ptr model, const std::string &amp;amp;endpointName, const Parameters ¶meters)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="421.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-83" value="+Eigen::VectorXd resolve_endpoint_motion(const Eigen::Vector &amp;amp;endpointMotion)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="444.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-84" value="+Eigen::VectorXd resolve_endpoint_twist(const Eigen::Vector &amp;amp;twist)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="467.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-85" value="+Eigen::VectorXd track_endpoint_trajectory(const Pose &amp;amp;desiredPose, const Eigen::Vector &amp;amp;desiredVelocity, const Eigen::Vector &amp;amp;desiredAcceleration)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="490.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-86" value="+Eigen::VectorXd track_joint_trajectory(const Eigen::VectorXd &amp;desiredPosition, const Eigen::VectorXd &amp;desiredVelocity, const Eigen::VectorXd &amp;desiredAcceleration)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="513.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-87" value="+double manipulability() const" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="536.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-88" value="+Eigen::VectorXd manipulability_gradient()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="559.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-89" value="+Pose endpoint_pose() const" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="582.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-90" value="+Eigen::Vector endpoint_velocity() const" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="605.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-91" value="+Eigen::Matrix jacobian() const" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="628.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-92" value="+void update()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="651.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-93" value="+void set_control_parameters(const Parameters &amp;parameters)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="674.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-94" value="+bool set_redundant_task(const Eigen::VectorXd &amp;task)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="697.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-95" value="+bool is_singular()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="720.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-96" value="+std::shared_ptr model() const" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="743.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-97" value="+double frequency() const" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry y="766.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-98" value="&lt;&lt;abstract&gt;&gt;&#xa;QPSolver" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=46.5;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="20" y="50" width="106" height="85.5" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-99" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-98">
          <mxGeometry y="46.5" width="106" height="8" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-100" value="+void solve()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-98">
          <mxGeometry y="54.5" width="106" height="31" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-101" value="KinematicTree" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=28.4;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="176" y="61" width="258" height="64.4" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-102" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-101">
          <mxGeometry y="28.4" width="258" height="8" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-103" value="+Eigen::VectorXd joint_velocities()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-101">
          <mxGeometry y="36.4" width="258" height="28" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-104" value="Pose" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=26.571428571428573;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="484" y="50" width="225" height="88.57142857142857" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-105" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-104">
          <mxGeometry y="26.571428571428573" width="225" height="8" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-106" value="+Eigen::VectorXd position()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-104">
          <mxGeometry y="34.57142857142857" width="225" height="27" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-107" value="+Eigen::Matrix3d orientation()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-104">
          <mxGeometry y="61.57142857142857" width="225" height="27" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-108" value="ReferenceFrame" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=28.4;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="760" y="61" width="288" height="64.4" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-109" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-108">
          <mxGeometry y="28.4" width="288" height="8" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-110" value="+Eigen::Matrix transformation_matrix()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-108">
          <mxGeometry y="36.4" width="288" height="28" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-111" value="Limits" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=26.571428571428573;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="1097" y="50" width="155" height="93.57142857142857" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-112" value="+double lower_limit" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-111">
          <mxGeometry y="26.571428571428573" width="155" height="27" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-113" value="+double upper_limit" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-111">
          <mxGeometry y="53.57142857142857" width="155" height="27" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-114" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-111">
          <mxGeometry y="80.57142857142857" width="155" height="13" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-115" value="Parameters" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=26.571428571428573;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="1302" y="50" width="159" height="93.57142857142857" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-116" value="+double controlGain" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-115">
          <mxGeometry y="26.571428571428573" width="159" height="27" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-117" value="+double frequency" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-115">
          <mxGeometry y="53.57142857142857" width="159" height="27" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-118" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-115">
          <mxGeometry y="80.57142857142857" width="159" height="13" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-119" value="" style="curved=1;startArrow=block;startSize=16;startFill=0;endArrow=none;exitX=0.5007001768867925;exitY=1;entryX=0.000032715661641541036;entryY=0.030882908209860917;rounded=0;" edge="1" parent="1" source="cwrhPxI-1V7dXrqJGJlZ-98" target="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="73" y="176" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-120" value="" style="curved=1;startArrow=block;startSize=16;startFill=0;endArrow=none;exitX=0.5010446947674418;exitY=1;entryX=0.14989967799305653;entryY=0;rounded=0;" edge="1" parent="1" source="cwrhPxI-1V7dXrqJGJlZ-101" target="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="305" y="176" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-121" value="" style="curved=1;startArrow=block;startSize=16;startFill=0;endArrow=none;exitX=0.5024652777777778;exitY=1;entryX=0.37960741771583;entryY=0;rounded=0;" edge="1" parent="1" source="cwrhPxI-1V7dXrqJGJlZ-104" target="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="597" y="176" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-122" value="" style="curved=1;startArrow=block;startSize=16;startFill=0;endArrow=none;exitX=0.4982096354166667;exitY=1;entryX=0.6208440584148233;entryY=0;rounded=0;" edge="1" parent="1" source="cwrhPxI-1V7dXrqJGJlZ-108" target="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="903" y="176" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-123" value="" style="curved=1;startArrow=block;startSize=16;startFill=0;endArrow=none;exitX=0.5020917338709677;exitY=1;entryX=0.8344562345981106;entryY=0;rounded=0;" edge="1" parent="1" source="cwrhPxI-1V7dXrqJGJlZ-111" target="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="1175" y="176" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-124" value="" style="curved=1;startArrow=block;startSize=16;startFill=0;endArrow=none;exitX=0.5014249213836478;exitY=1;entryX=0.9973400164085734;entryY=0;rounded=0;" edge="1" parent="1" source="cwrhPxI-1V7dXrqJGJlZ-115" target="cwrhPxI-1V7dXrqJGJlZ-63">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="1382" y="176" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-125" value="SerialLinkBase" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=22.794117647058822;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="153" y="201" width="1194" height="789.7941176470588" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-126" value="-bool _redundantTaskSet" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="22.794117647058822" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-127" value="-double _jointPositionGain" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="45.794117647058826" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-128" value="-double _jointVelocityGain" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="68.79411764705883" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-129" value="-double _manipulability" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="91.79411764705883" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-130" value="-double _minManipulability" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="114.79411764705883" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-131" value="-double _maxJointAcceleration" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="137.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-132" value="-double _controlFrequency" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="160.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-133" value="-Eigen::Matrix _cartesianStiffness" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="183.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-134" value="-Eigen::Matrix _cartesianDamping" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="206.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-135" value="-Eigen::Matrix _jacobianMatrix" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="229.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-136" value="-Eigen::Matrix _forceEllipsoid" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="252.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-137" value="-Eigen::Matrix _constraintMatrix" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="275.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-138" value="-Eigen::VectorXd _constraintVector" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="298.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-139" value="-Eigen::VectorXd _redundantTask" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="321.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-140" value="-std::shared_ptr _model" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="344.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-141" value="-Pose _endpointPose" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="367.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-142" value="-ReferenceFrame *_endpointFrame" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="390.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-143" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="413.79411764705884" width="1194" height="8" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-144" value="+SerialLinkBase(std::shared_ptr model, const std::string &amp;amp;endpointName, const Parameters ¶meters)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="421.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-145" value="+Eigen::VectorXd resolve_endpoint_motion(const Eigen::Vector &amp;amp;endpointMotion)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="444.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-146" value="+Eigen::VectorXd resolve_endpoint_twist(const Eigen::Vector &amp;amp;twist)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="467.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-147" value="+Eigen::VectorXd track_endpoint_trajectory(const Pose &amp;amp;desiredPose, const Eigen::Vector &amp;amp;desiredVelocity, const Eigen::Vector &amp;amp;desiredAcceleration)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="490.79411764705884" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-148" value="+Eigen::VectorXd track_joint_trajectory(const Eigen::VectorXd &amp;desiredPosition, const Eigen::VectorXd &amp;desiredVelocity, const Eigen::VectorXd &amp;desiredAcceleration)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="513.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-149" value="+double manipulability() const" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="536.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-150" value="+Eigen::VectorXd manipulability_gradient()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="559.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-151" value="+Pose endpoint_pose() const" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="582.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-152" value="+Eigen::Vector endpoint_velocity() const" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="605.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-153" value="+Eigen::Matrix jacobian() const" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="628.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-154" value="+void update()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="651.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-155" value="+void set_control_parameters(const Parameters &amp;parameters)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="674.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-156" value="+bool set_redundant_task(const Eigen::VectorXd &amp;task)" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="697.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-157" value="+bool is_singular()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="720.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-158" value="+std::shared_ptr model() const" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="743.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-159" value="+double frequency() const" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry y="766.7941176470588" width="1194" height="23" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-160" value="&lt;&lt;abstract&gt;&gt;&#xa;QPSolver" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=46.5;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="20" y="50" width="106" height="85.5" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-161" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-160">
          <mxGeometry y="46.5" width="106" height="8" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-162" value="+void solve()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-160">
          <mxGeometry y="54.5" width="106" height="31" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-163" value="KinematicTree" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=28.4;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="176" y="61" width="258" height="64.4" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-164" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-163">
          <mxGeometry y="28.4" width="258" height="8" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-165" value="+Eigen::VectorXd joint_velocities()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-163">
          <mxGeometry y="36.4" width="258" height="28" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-166" value="Pose" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=26.571428571428573;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="484" y="50" width="225" height="88.57142857142857" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-167" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-166">
          <mxGeometry y="26.571428571428573" width="225" height="8" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-168" value="+Eigen::VectorXd position()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-166">
          <mxGeometry y="34.57142857142857" width="225" height="27" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-169" value="+Eigen::Matrix3d orientation()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-166">
          <mxGeometry y="61.57142857142857" width="225" height="27" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-170" value="ReferenceFrame" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=28.4;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="760" y="61" width="288" height="64.4" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-171" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-170">
          <mxGeometry y="28.4" width="288" height="8" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-172" value="+Eigen::Matrix transformation_matrix()" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-170">
          <mxGeometry y="36.4" width="288" height="28" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-173" value="Limits" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=26.571428571428573;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="1097" y="50" width="155" height="93.57142857142857" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-174" value="+double lower_limit" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-173">
          <mxGeometry y="26.571428571428573" width="155" height="27" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-175" value="+double upper_limit" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-173">
          <mxGeometry y="53.57142857142857" width="155" height="27" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-176" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-173">
          <mxGeometry y="80.57142857142857" width="155" height="13" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-177" value="Parameters" style="swimlane;fontStyle=1;align=center;verticalAlign=top;childLayout=stackLayout;horizontal=1;startSize=26.571428571428573;horizontalStack=0;resizeParent=1;resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;" vertex="1" parent="1">
          <mxGeometry x="1302" y="50" width="159" height="93.57142857142857" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-178" value="+double controlGain" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-177">
          <mxGeometry y="26.571428571428573" width="159" height="27" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-179" value="+double frequency" style="text;strokeColor=none;fillColor=none;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;overflow=hidden;rotatable=0;points=[[0,0.5],[1,0.5]];portConstraint=eastwest;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-177">
          <mxGeometry y="53.57142857142857" width="159" height="27" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-180" style="line;strokeWidth=1;fillColor=none;align=left;verticalAlign=middle;spacingTop=-1;spacingLeft=3;spacingRight=3;rotatable=0;labelPosition=right;points=[];portConstraint=eastwest;strokeColor=inherit;" vertex="1" parent="cwrhPxI-1V7dXrqJGJlZ-177">
          <mxGeometry y="80.57142857142857" width="159" height="13" as="geometry" />
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-181" value="" style="curved=1;startArrow=block;startSize=16;startFill=0;endArrow=none;exitX=0.5007001768867925;exitY=1;entryX=0.000032715661641541036;entryY=0.030882908209860917;rounded=0;" edge="1" parent="1" source="cwrhPxI-1V7dXrqJGJlZ-160" target="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="73" y="176" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-182" value="" style="curved=1;startArrow=block;startSize=16;startFill=0;endArrow=none;exitX=0.5010446947674418;exitY=1;entryX=0.14989967799305653;entryY=0;rounded=0;" edge="1" parent="1" source="cwrhPxI-1V7dXrqJGJlZ-163" target="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="305" y="176" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-183" value="" style="curved=1;startArrow=block;startSize=16;startFill=0;endArrow=none;exitX=0.5024652777777778;exitY=1;entryX=0.37960741771583;entryY=0;rounded=0;" edge="1" parent="1" source="cwrhPxI-1V7dXrqJGJlZ-166" target="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="597" y="176" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-184" value="" style="curved=1;startArrow=block;startSize=16;startFill=0;endArrow=none;exitX=0.4982096354166667;exitY=1;entryX=0.6208440584148233;entryY=0;rounded=0;" edge="1" parent="1" source="cwrhPxI-1V7dXrqJGJlZ-170" target="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="903" y="176" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-185" value="" style="curved=1;startArrow=block;startSize=16;startFill=0;endArrow=none;exitX=0.5020917338709677;exitY=1;entryX=0.8344562345981106;entryY=0;rounded=0;" edge="1" parent="1" source="cwrhPxI-1V7dXrqJGJlZ-173" target="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="1175" y="176" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="cwrhPxI-1V7dXrqJGJlZ-186" value="" style="curved=1;startArrow=block;startSize=16;startFill=0;endArrow=none;exitX=0.5014249213836478;exitY=1;entryX=0.9973400164085734;entryY=0;rounded=0;" edge="1" parent="1" source="cwrhPxI-1V7dXrqJGJlZ-177" target="cwrhPxI-1V7dXrqJGJlZ-125">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="1382" y="176" />
            </Array>
          </mxGeometry>
        </mxCell>
      </root>
    </mxGraphModel>
  </diagram>
</mxfile>
