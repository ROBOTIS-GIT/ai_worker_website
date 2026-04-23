# Open Source

## Common Open Source

### CYCLO
Cyclo is a modular, open-source framework by ROBOTIS designed to make building and operating Physical AI systems accessible, from AI integration to robot operations.
| Resource | Description | Link |
|----------|-------------|------|
| **CYCLO** | Modular framework for Physical AI systems | [GitHub](https://github.com/ROBOTIS-GIT/cyclo) |
| **CYCLO Control** | Control packages for the Physical AI lineup | [GitHub](https://github.com/ROBOTIS-GIT/cyclo_control) |

### Shared Resources

| Resource | Description | Link |
|----------|-------------|------|
| **Physical AI Tools** | ROS 2 packages for generating datasets in LeRobotDataset format | [GitHub](https://github.com/ROBOTIS-GIT/physical_ai_tools) |
| **AI Models & Datasets** | Pre-trained models and training datasets | [Hugging Face](https://huggingface.co/ROBOTIS) |
| **DYNAMIXEL SDK** | Control SDK for DYNAMIXEL actuators and platforms | [GitHub](https://github.com/ROBOTIS-GIT/DynamixelSDK) |
| **DYNAMIXEL Hardware Interface** | `ros2_control` hardware interface for DYNAMIXEL motors | [GitHub](https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface) |
| **ROBOTIS Interfaces** | Shared interface definitions for the Physical AI lineup | [GitHub](https://github.com/ROBOTIS-GIT/robotis_interfaces) |
| **ROBOTIS Applications** | Application-level ROS 2 integrations and dependencies | [GitHub](https://github.com/ROBOTIS-GIT/robotis_applications) |
| **Isaac Sim Models** | USD robot assets for Isaac Sim / Isaac Lab | [GitHub](https://github.com/ROBOTIS-GIT/robotis_lab/tree/main/source/robotis_lab/data/robots) |
| **MuJoCo Models** | MuJoCo menagerie models | [GitHub](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie) |

## Product-specific Open Source

<table>
  <thead>
    <tr>
      <th>Product</th>
      <th>Resource</th>
      <th>Description</th>
      <th>Link</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td rowspan="5"><strong>AI Worker</strong></td>
      <td><strong>AI Worker</strong></td>
      <td>ROS 2 Packages for operating<br>the AI Worker</td>
      <td><a href="https://github.com/ROBOTIS-GIT/ai_worker">GitHub</a></td>
    </tr>
    <tr>
      <td><strong>Simulation<br>Models</strong></td>
      <td>URDF(ROS), MJCF(MuJoCo)<br>and USD(Isaac Sim/Lab) model files for <br> simulation</td>
      <td>
        <a href="https://github.com/ROBOTIS-GIT/ai_worker/tree/main/ffw_description/urdf">URDF</a><br>
        <a href="https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie">MJCF</a><br>
        <a href="https://github.com/ROBOTIS-GIT/robotis_lab/tree/main/source/robotis_lab/data/robots">USD</a>
      </td>
    </tr>
    <tr>
      <td><strong>Drawing<br>Files</strong></td>
      <td>Hardware design files and<br>technical drawings</td>
      <td>
        <strong>FFW-BG2:</strong><br>
        <a href="https://www.robotis.com/service/download.php?no=2214">PDF</a><br>
        <a href="https://www.robotis.com/service/download.php?no=2231">DWG</a> / <a href="https://www.robotis.com/service/download.php?no=2227">STEP</a><br>
        <strong>FFW-SG2:</strong><br>
        <a href="https://www.robotis.com/service/download.php?no=2215">PDF</a><br>
        <a href="https://www.robotis.com/service/download.php?no=2232">DWG</a> / <a href="https://www.robotis.com/service/download.php?no=2228">STEP</a><br>
        <strong>FFW-LG2:</strong><br>
        <a href="https://www.robotis.com/service/download.php?no=2229">STEP</a>
      </td>
    </tr>
    <tr>
      <td><strong>Docker<br>Images</strong></td>
      <td>Ready-to-use development<br>environments</td>
      <td><a href="https://hub.docker.com/r/robotis/ai-worker">Docker Hub</a></td>
    </tr>
    <tr>
      <td><strong>Tutorial<br>Videos</strong></td>
      <td>Educational content and demonstrations</td>
      <td><a href="https://www.youtube.com/@ROBOTISOpenSourceTeam">YouTube</a></td>
    </tr>
    <tr>
      <td rowspan="5"><strong>OMY</strong></td>
      <td><strong>AI<br>Manipulator</strong></td>
      <td>ROS 2 Packages for operating<br>the AI Manipulator</td>
      <td><a href="https://github.com/ROBOTIS-GIT/open_manipulator">GitHub</a></td>
    </tr>
    <tr>
      <td><strong>Simulation<br>Models</strong></td>
      <td>URDF(ROS), MJCF(MuJoCo)<br>and USD(Isaac Sim/Lab) model files for <br> simulation</td>
      <td>
        <a href="https://github.com/ROBOTIS-GIT/open_manipulator/tree/main/open_manipulator_description/urdf">URDF</a><br>
        <a href="https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie">MJCF</a><br>
        <a href="https://github.com/ROBOTIS-GIT/robotis_lab/tree/main/source/robotis_lab/data/robots">USD</a>
      </td>
    </tr>
    <tr>
      <td><strong>Drawing<br>Files</strong></td>
      <td>Hardware design files and<br>technical drawings</td>
      <td>
        <strong>OMY-F3M:</strong><br>
        <a href="https://www.robotis.com/service/download.php?no=2217">PDF</a><br>
        <a href="https://www.robotis.com/service/download.php?no=2216">DWG</a> / <a href="https://www.robotis.com/service/download.php?no=2218">STEP</a><br>
        <strong>OMY-L100:</strong><br>
        <a href="https://www.robotis.com/service/download.php?no=2220">PDF</a><br>
        <a href="https://www.robotis.com/service/download.php?no=2219">DWG</a> / <a href="https://www.robotis.com/service/download.php?no=2221">STEP</a>
      </td>
    </tr>
    <tr>
      <td><strong>Docker<br>Images</strong></td>
      <td>Ready-to-use development<br>environments</td>
      <td><a href="https://hub.docker.com/r/robotis/open-manipulator">Docker Hub</a></td>
    </tr>
    <tr>
      <td><strong>Tutorial<br>Videos</strong></td>
      <td>Educational content and demonstrations</td>
      <td><a href="https://www.youtube.com/@ROBOTISOpenSourceTeam">YouTube</a></td>
    </tr>
    <tr>
      <td rowspan="5"><strong>OMX</strong></td>
      <td><strong>AI<br>Manipulator</strong></td>
      <td>ROS 2 Packages for operating<br>the AI Manipulator</td>
      <td><a href="https://github.com/ROBOTIS-GIT/open_manipulator">GitHub</a></td>
    </tr>
    <tr>
      <td><strong>Simulation<br>Models</strong></td>
      <td>URDF(ROS), MJCF(MuJoCo)<br>and USD(Isaac Sim/Lab) model files for <br> simulation</td>
      <td>
        <a href="https://github.com/ROBOTIS-GIT/open_manipulator/tree/main/open_manipulator_description/urdf">URDF</a><br>
        <a href="https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie">MJCF</a><br>
        <a href="https://github.com/ROBOTIS-GIT/robotis_lab/tree/main/source/robotis_lab/data/robots">USD</a>
      </td>
    </tr>
    <tr>
      <td><strong>Drawing<br>Files</strong></td>
      <td>Hardware design files and<br>technical drawings</td>
      <td>
        <strong>OMX-F:</strong><br>
        <a href="https://www.robotis.com/service/download.php?no=2223">PDF</a> / <a href="https://www.robotis.com/service/download.php?no=2224">STEP</a><br>
        <strong>OMX-L:</strong><br>
        <a href="https://www.robotis.com/service/download.php?no=2225">PDF</a> / <a href="https://www.robotis.com/service/download.php?no=2226">STEP</a>
      </td>
    </tr>
    <tr>
      <td><strong>Docker<br>Images</strong></td>
      <td>Ready-to-use development<br>environments</td>
      <td><a href="https://hub.docker.com/r/robotis/open-manipulator">Docker Hub</a></td>
    </tr>
    <tr>
      <td><strong>Tutorial<br>Videos</strong></td>
      <td>Educational content and demonstrations</td>
      <td><a href="https://www.youtube.com/@ROBOTISOpenSourceTeam">YouTube</a></td>
    </tr>
    <tr>
      <td rowspan="5"><strong>ROBOTIS Hand</strong></td>
      <td><strong>ROBOTIS<br>Hand</strong></td>
      <td>ROS 2 Packages for operating<br>the ROBOTIS Hand</td>
      <td><a href="https://github.com/ROBOTIS-GIT/robotis_hand">GitHub</a></td>
    </tr>
    <tr>
      <td><strong>Simulation<br>Models</strong></td>
      <td>URDF(ROS) model file for simulation</td>
      <td><a href="https://github.com/ROBOTIS-GIT/robotis_hand/tree/main/robotis_hand_description/urdf">URDF</a></td>
    </tr>
    <tr>
      <td><strong>Drawing<br>Files</strong></td>
      <td>Hardware design files and<br>technical drawings</td>
      <td>
        <strong>HX5-D20-MRT:</strong><br>
        <a href="https://www.robotis.com/service/download.php?no=2239">PDF</a><br>
        <a href="https://www.robotis.com/service/download.php?no=2238">DWG</a> / <a href="https://www.robotis.com/service/download.php?no=2240">STEP</a><br>
        <strong>HX5-D20-MLT:</strong><br>
        <a href="https://www.robotis.com/service/download.php?no=2242">PDF</a><br>
        <a href="https://www.robotis.com/service/download.php?no=2241">DWG</a> / <a href="https://www.robotis.com/service/download.php?no=2243">STEP</a>
      </td>
    </tr>
    <tr>
      <td><strong>Docker<br>Images</strong></td>
      <td>Ready-to-use development<br>environments</td>
      <td><a href="https://hub.docker.com/r/robotis/robotis-hand">Docker Hub</a></td>
    </tr>
    <tr>
      <td><strong>Tutorial<br>Videos</strong></td>
      <td>Educational content and demonstrations</td>
      <td><a href="https://www.youtube.com/@ROBOTISOpenSourceTeam">YouTube</a></td>
    </tr>
  </tbody>
</table>

We welcome contributions from the community to help improve and expand these open-source ecosystems.
