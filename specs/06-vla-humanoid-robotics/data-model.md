# Data Model: Vision-Language-Action (VLA) Module

## Entity Models

### VLACommand
**Description:** Represents a command generated from natural language input that integrates vision, language, and action components.

**Fields:**
- id (string): Unique identifier for the command
- natural_language_input (string): Original natural language command from user
- parsed_intent (string): Parsed intent from the natural language
- vision_context (object): Visual information relevant to the command
  - object_detections (array): Detected objects in the scene
  - spatial_relations (array): Spatial relationships between objects
  - scene_description (string): Description of current scene
- action_sequence (array): Sequence of actions to execute
  - action_type (string): Type of action (move, grasp, navigate, etc.)
  - parameters (object): Action-specific parameters
  - constraints (object): Safety and execution constraints
- confidence_score (number): Confidence level of the interpretation
- timestamp (datetime): When the command was generated

**Relationships:**
- One-to-many with ActionExecution (command may result in multiple executions)
- Many-to-one with VLAContext (command belongs to a specific context)

### ActionExecution
**Description:** Represents the execution of a specific action within the VLA framework.

**Fields:**
- id (string): Unique identifier for the execution
- command_id (string): Reference to the parent VLACommand
- action_type (string): Type of action being executed
- parameters (object): Parameters for the action
- status (string): Current status (pending, executing, completed, failed)
- execution_log (array): Log of execution steps and results
- robot_feedback (object): Robot's sensory feedback during execution
- success_probability (number): Estimated probability of success
- timestamp (datetime): When execution started

**Relationships:**
- Many-to-one with VLACommand (execution belongs to a command)
- One-to-many with ExecutionFeedback (one execution may generate multiple feedbacks)

### ExecutionFeedback
**Description:** Feedback from the robot system during action execution.

**Fields:**
- id (string): Unique identifier for the feedback
- execution_id (string): Reference to the parent ActionExecution
- feedback_type (string): Type of feedback (visual, tactile, proprioceptive, etc.)
- feedback_data (object): Specific feedback information
- timestamp (datetime): When feedback was recorded
- relevance_score (number): How relevant the feedback is to the task

**Relationships:**
- Many-to-one with ActionExecution (feedback belongs to an execution)

### VLAContext
**Description:** Contextual information that guides the interpretation and execution of VLA commands.

**Fields:**
- id (string): Unique identifier for the context
- environment_map (object): Current map of the environment
- robot_state (object): Current state of the robot
  - position (object): x, y, z coordinates and orientation
  - joint_states (array): States of all robot joints
  - battery_level (number): Current battery level
- task_history (array): Previous tasks and their outcomes
- user_preferences (object): User preferences for interaction
- safety_constraints (object): Current safety parameters

**Relationships:**
- One-to-many with VLACommand (context may generate multiple commands)

### LLMInteraction
**Description:** Record of interactions with the large language model for VLA processing.

**Fields:**
- id (string): Unique identifier for the interaction
- input_text (string): Text sent to the LLM
- output_text (string): Text received from the LLM
- model_name (string): Name of the LLM used
- prompt_template (string): Template used for the interaction
- tokens_used (number): Number of tokens in the interaction
- response_time (number): Time taken for the response in milliseconds
- confidence (number): Confidence level of the LLM's output
- timestamp (datetime): When the interaction occurred

**Relationships:**
- One-to-one with VLACommand (each command may have one primary LLM interaction)

### TaskPlan
**Description:** Structured plan for executing complex tasks in the VLA framework.

**Fields:**
- id (string): Unique identifier for the task plan
- original_command (string): Original natural language command
- subtasks (array): Array of subtasks that make up the plan
  - subtask_id (string): Unique identifier for the subtask
  - action_sequence (array): Actions required for the subtask
  - dependencies (array): Other subtasks this subtask depends on
  - success_criteria (string): Criteria for considering the subtask complete
- execution_strategy (string): Overall strategy for executing the plan
- estimated_completion_time (number): Estimated time to complete the plan in seconds
- current_progress (object): Current state of plan execution
- status (string): Current status (planning, executing, completed, failed)

**Relationships:**
- One-to-many with SubTask (plan consists of multiple subtasks)
- One-to-one with VLACommand (plan is generated from a command)

### VisionPercept
**Description:** Representation of visual information processed by the VLA system.

**Fields:**
- id (string): Unique identifier for the percept
- timestamp (datetime): When the percept was captured
- image_data (string): Reference to the captured image
- object_detections (array): Array of detected objects
  - object_class (string): Class of the detected object
  - confidence (number): Confidence of the detection
  - bounding_box (object): Coordinates of the bounding box
- spatial_relations (array): Relationships between objects
- scene_description (string): Natural language description of the scene
- semantic_labels (array): Semantic segmentation labels
- depth_data (object): Depth information if available

**Relationships:**
- One-to-many with VLACommand (percepts inform commands)

## State Transitions

### VLACommand State Transitions
- `new` → `parsed` (when natural language is processed)
- `parsed` → `validated` (when safety and feasibility checks pass)
- `validated` → `executing` (when action execution begins)
- `executing` → `completed` (when all actions complete successfully)
- `executing` → `failed` (when an action fails)
- `executing` → `interrupted` (when user interrupts or safety issue occurs)

### ActionExecution State Transitions
- `pending` → `executing` (when action execution starts)
- `executing` → `completed` (when action completes successfully)
- `executing` → `failed` (when action fails)
- `executing` → `paused` (when action needs to wait for conditions)
- `paused` → `executing` (when action resumes)
- `executing` → `canceled` (when action is canceled)

## Validation Rules

### VLACommand Validation
- `natural_language_input` must not be empty
- `confidence_score` must be between 0 and 1
- `action_sequence` must contain at least one action
- `parsed_intent` must match a predefined list of valid intents

### ActionExecution Validation
- `status` must be one of: pending, executing, completed, failed, paused, canceled
- `action_type` must be a valid ROS 2 action type
- `success_probability` must be between 0 and 1

### ExecutionFeedback Validation
- `feedback_type` must be one of: visual, tactile, proprioceptive, audio
- `relevance_score` must be between 0 and 1

### TaskPlan Validation
- `status` must be one of: planning, executing, completed, failed
- `estimated_completion_time` must be a positive number

## Relationships and Constraints

1. **VLACommand-ActionExecution**: One VLACommand can result in multiple ActionExecutions
2. **ActionExecution-ExecutionFeedback**: One ActionExecution can generate multiple ExecutionFeedback entries
3. **VLACommand-VLAContext**: Each command must be associated with a context
4. **TaskPlan-VLACommand**: Each task plan is generated from exactly one VLACommand
5. **VisionPercept-VLACommand**: Multiple percepts can inform a single command

## API Contract Considerations

The data models above would translate to the following API endpoints for educational examples:

1. `POST /v1/vla/commands` - Submit a natural language command
2. `GET /v1/vla/commands/{id}` - Get status of a VLA command
3. `POST /v1/vla/execute` - Execute a parsed VLA command
4. `GET /v1/vla/context` - Get current VLA context
5. `POST /v1/vla/plan` - Generate a task plan from natural language