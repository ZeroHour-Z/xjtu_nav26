from typing import Optional

import py_trees
from py_trees.common import Status
from rclpy.node import Node
from std_msgs.msg import Bool, String

from ..registry import register


@register("TopicBoolCondition")
class TopicBoolCondition(py_trees.behaviour.Behaviour):
	def __init__(
		self,
		name: str,
		node: Node,
		topic: str,
		expected: bool = True,
		latched_ok: bool = True,
		default_value: Optional[bool] = None,
	):
		super().__init__(name)
		self.node = node
		self.topic = topic
		self.expected = expected
		self.latched_ok = latched_ok
		self._last_value: Optional[bool] = None if default_value is None else bool(default_value)
		self._sub = None

	def setup(self, **kwargs) -> None:
		# kwargs may contain: node, timeout, visitor etc. We only need self.node here.
		if self._sub is None:
			self._sub = self.node.create_subscription(Bool, self.topic, self._on_msg, 10)

	def initialise(self) -> None:
		pass

	def update(self) -> Status:
		if self._last_value is None:
			return Status.RUNNING
		return Status.SUCCESS if (self._last_value == self.expected) else Status.FAILURE

	def terminate(self, new_status: Status) -> None:
		pass

	def _on_msg(self, msg: Bool) -> None:
		if self.latched_ok and msg.data == self.expected:
			self._last_value = self.expected
			return
		self._last_value = msg.data


@register("TopicNumericThreshold")
class TopicNumericThreshold(py_trees.behaviour.Behaviour):
	def __init__(
		self,
		name: str,
		node: Node,
		topic: str,
		threshold: float,
		op: str = "<",
		msg_type: str = "std_msgs/msg/Float32",
		field: str = "data",
		latched_ok: bool = False,
	):
		super().__init__(name)
		self.node = node
		self.topic = topic
		self.threshold = float(threshold)
		self.op = str(op)
		self.msg_type = str(msg_type)
		self.field = str(field)
		self.latched_ok = bool(latched_ok)
		self._value: Optional[float] = None
		self._sub = None
		self._latched_success: bool = False

	def setup(self, **kwargs) -> None:
		if self._sub is not None:
			return
		msg_cls = self._resolve_msg_type(self.msg_type)
		self._sub = self.node.create_subscription(msg_cls, self.topic, self._on_msg, 10)

	def initialise(self) -> None:
		# clear latched only on initialise if not configured to latch
		if not self.latched_ok:
			self._latched_success = False

	def update(self) -> Status:
		if self._latched_success:
			return Status.SUCCESS
		if self._value is None:
			return Status.RUNNING
		ok = self._compare(self._value, self.op, self.threshold)
		if ok and self.latched_ok:
			self._latched_success = True
		return Status.SUCCESS if ok else Status.FAILURE

	def terminate(self, new_status: Status) -> None:
		pass

	def _on_msg(self, msg) -> None:
		try:
			val = self._extract_field_value(msg, self.field)
			self._value = float(val)
		except Exception as exc:
			self.node.get_logger().warn(f"TopicNumericThreshold parse error on {self.topic}: {exc}")

	@staticmethod
	def _resolve_msg_type(type_str: str):
		import importlib
		s = type_str.strip()
		module_path: Optional[str] = None
		class_name: Optional[str] = None
		if "/" in s:
			parts = s.split("/")
			# Accept forms like "std_msgs/msg/Float32" or "std_msgs/Float32"
			if len(parts) == 3:
				module_path = f"{parts[0]}.{parts[1]}"
				class_name = parts[2]
			elif len(parts) == 2:
				module_path = f"{parts[0]}.msg"
				class_name = parts[1]
			else:
				raise ValueError(f"Unrecognized msg_type format: {s}")
		elif "." in s:
			module_path, class_name = s.rsplit(".", 1)
		else:
			raise ValueError(f"Unrecognized msg_type format: {s}")
		mod = importlib.import_module(module_path)
		return getattr(mod, class_name)

	@staticmethod
	def _extract_field_value(msg, field_path: str):
		obj = msg
		for attr in field_path.split("."):
			obj = getattr(obj, attr)
		return obj

	@staticmethod
	def _compare(value: float, op: str, threshold: float) -> bool:
		if op == "<":
			return value < threshold
		if op == "<=":
			return value <= threshold
		if op == ">":
			return value > threshold
		if op == ">=":
			return value >= threshold
		if op == "==":
			return value == threshold
		if op == "!=":
			return value != threshold
		raise ValueError(f"Unsupported operator: {op}")


@register("ParamNumericThreshold")
class ParamNumericThreshold(py_trees.behaviour.Behaviour):
	def __init__(self, name: str, node: Node, param: str, threshold: float, op: str = "<", latched_ok: bool = False):
		super().__init__(name)
		self.node = node
		self.param = param.lstrip("/")
		self.threshold = float(threshold)
		self.op = str(op)
		self.latched_ok = bool(latched_ok)
		self._latched_success: bool = False

	def setup(self, **kwargs) -> None:
		# Declare parameter if it does not exist to allow external updates
		try:
			if not self.node.has_parameter(self.param):
				# Choose a safe default value based on operator so it won't trigger accidentally
				default = 1e9 if self.op in ("<", "<=") else -1e9 if self.op in (">", ">=") else 0.0
				self.node.declare_parameter(self.param, float(default))
		except Exception:
			pass

	def initialise(self) -> None:
		if not self.latched_ok:
			self._latched_success = False

	def update(self) -> Status:
		if self._latched_success:
			return Status.SUCCESS
		try:
			param_value = self.node.get_parameter(self.param).value
		except Exception:
			return Status.RUNNING
		if param_value is None:
			return Status.RUNNING
		ok = TopicNumericThreshold._compare(float(param_value), self.op, self.threshold)
		if ok and self.latched_ok:
			self._latched_success = True
		return Status.SUCCESS if ok else Status.FAILURE


@register("WaitForNumericThreshold")
class WaitForNumericThreshold(py_trees.behaviour.Behaviour):
	def __init__(
		self,
		name: str,
		node: Node,
		topic: str,
		threshold: float,
		op: str = ">=",
		msg_type: str = "std_msgs/msg/Float32",
		field: str = "data",
	):
		super().__init__(name)
		self.node = node
		self.topic = topic
		self.threshold = float(threshold)
		self.op = str(op)
		self.msg_type = str(msg_type)
		self.field = str(field)
		self._value: Optional[float] = None
		self._sub = None

	def setup(self, **kwargs) -> None:
		if self._sub is not None:
			return
		msg_cls = TopicNumericThreshold._resolve_msg_type(self.msg_type)
		self._sub = self.node.create_subscription(msg_cls, self.topic, self._on_msg, 10)

	def initialise(self) -> None:
		pass

	def update(self) -> py_trees.common.Status:
		if self._value is None:
			return py_trees.common.Status.RUNNING
		ok = TopicNumericThreshold._compare(self._value, self.op, self.threshold)
		return py_trees.common.Status.SUCCESS if ok else py_trees.common.Status.RUNNING

	def terminate(self, new_status: py_trees.common.Status) -> None:
		pass

	def _on_msg(self, msg) -> None:
		try:
			val = TopicNumericThreshold._extract_field_value(msg, self.field)
			self._value = float(val)
		except Exception as exc:
			self.node.get_logger().warn(f"WaitForNumericThreshold parse error on {self.topic}: {exc}")


@register("WaitForParamNumericThreshold")
class WaitForParamNumericThreshold(py_trees.behaviour.Behaviour):
	def __init__(self, name: str, node: Node, param: str, threshold: float, op: str = ">="):
		super().__init__(name)
		self.node = node
		self.param = param.lstrip("/")
		self.threshold = float(threshold)
		self.op = str(op)

	def setup(self, **kwargs) -> None:
		try:
			if not self.node.has_parameter(self.param):
				# Default so that condition is initially false
				default = -1e9 if self.op in (">", ">=") else 1e9 if self.op in ("<", "<=") else 0.0
				self.node.declare_parameter(self.param, float(default))
		except Exception:
			pass

	def update(self) -> Status:
		try:
			param_value = self.node.get_parameter(self.param).value
		except Exception:
			return Status.RUNNING
		if param_value is None:
			return Status.RUNNING
		ok = TopicNumericThreshold._compare(float(param_value), self.op, self.threshold)
		return Status.SUCCESS if ok else Status.RUNNING


@register("ParamBoolCondition")
class ParamBoolCondition(py_trees.behaviour.Behaviour):
	def __init__(self, name: str, node: Node, param: str, expected: bool = True, latched_ok: bool = False):
		super().__init__(name)
		self.node = node
		self.param = param.lstrip("/")
		self.expected = bool(expected)
		self.latched_ok = bool(latched_ok)
		self._latched_success: bool = False

	def setup(self, **kwargs) -> None:
		try:
			if not self.node.has_parameter(self.param):
				self.node.declare_parameter(self.param, False)
		except Exception:
			pass

	def initialise(self) -> None:
		if not self.latched_ok:
			self._latched_success = False

	def update(self) -> Status:
		if self._latched_success:
			return Status.SUCCESS
		try:
			param_value = self.node.get_parameter(self.param).value
		except Exception:
			return Status.RUNNING
		if param_value is None:
			return Status.RUNNING
		ok = bool(param_value) == self.expected
		if ok and self.latched_ok:
			self._latched_success = True
		return Status.SUCCESS if ok else Status.FAILURE


@register("WaitForParamBool")
class WaitForParamBool(py_trees.behaviour.Behaviour):
	def __init__(self, name: str, node: Node, param: str, expected: bool = True):
		super().__init__(name)
		self.node = node
		self.param = param.lstrip("/")
		self.expected = bool(expected)

	def setup(self, **kwargs) -> None:
		try:
			if not self.node.has_parameter(self.param):
				self.node.declare_parameter(self.param, not self.expected)
		except Exception:
			pass

	def update(self) -> Status:
		try:
			param_value = self.node.get_parameter(self.param).value
		except Exception:
			return Status.RUNNING
		if param_value is None:
			return Status.RUNNING
		return Status.SUCCESS if bool(param_value) == self.expected else Status.RUNNING 


@register("TopicStringEquals")
class TopicStringEquals(py_trees.behaviour.Behaviour):
	def __init__(self, name: str, node: Node, topic: str, expected: str, latched_ok: bool = True):
		super().__init__(name)
		self.node = node
		self.topic = topic
		self.expected = str(expected)
		self.latched_ok = bool(latched_ok)
		self._last_value: Optional[str] = None
		self._sub = None
		self._latched_success: bool = False

	def setup(self, **kwargs) -> None:
		if self._sub is None:
			self._sub = self.node.create_subscription(String, self.topic, self._on_msg, 10)

	def initialise(self) -> None:
		if not self.latched_ok:
			self._latched_success = False

	def update(self) -> Status:
		if self._latched_success:
			return Status.SUCCESS
		if self._last_value is None:
			return Status.RUNNING
		ok = (self._last_value == self.expected)
		if ok and self.latched_ok:
			self._latched_success = True
		return Status.SUCCESS if ok else Status.FAILURE

	def terminate(self, new_status: Status) -> None:
		pass

	def _on_msg(self, msg: String) -> None:
		val = msg.data
		if self.latched_ok and val == self.expected:
			self._last_value = self.expected
			self._latched_success = True
			return
		self._last_value = val 
