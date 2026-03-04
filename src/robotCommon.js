import * as THREE from 'three';

class _Robot {
	constructor(name, links, joints, constraints, actuators) {

		this.name = name;
		this.links = links;
		this.joints = joints;
		this.constraints = constraints;
		this.actuators = actuators;
		this.linkMap = new Map();
		this.jointMap = new Map();
		this.constraintMap = new Map();
		this.actuatorMap = new Map();

		this.robotModel = {};
		this.robotTopo = {};
		this.actuatorsTopo = {};

	}
}

class _Link {
	constructor(name, origin_translation, origin_orientation, visual) {
		this.name = name;
		this.origin_translation = new _Translation(origin_translation[0], origin_translation[1], origin_translation[2]);
		this.origin_orientation = new _Orientation(origin_orientation[0], origin_orientation[1], origin_orientation[2]);
		this.visual = new _Visual(visual.origin_translation, visual.origin_orientation, visual.geometry, visual.material);
	}
}

class _Joint {
	constructor(
		name,
		type,
		parent,
		child,
		origin_translation,
		origin_orientation,
		axis,
		limit
	) {
		this.name = name;
		this.type = type;
		this.parent = parent;
		this.child = child;
		this.origin_translation = new _Translation(origin_translation[0], origin_translation[1], origin_translation[2]);
		this.origin_orientation = new _Orientation(origin_orientation[0], origin_orientation[1], origin_orientation[2]);
		this.axis = new _Axis(axis[0], axis[1], axis[2]);
		this.limit = new _Limit(limit.lower, limit.upper);
	}
}
class _Constraint {
	constructor(
		name,
		type,
		jointL,
		jointA,
		jointB,
		jointL_base_angle,
		length,
		limit
	) {
		this.name = name;
		this.type = type;
		this.jointL = jointL;
		this.jointA = jointA;
		this.jointB = jointB;
		this.jointL_base_angle = jointL_base_angle;
		this.length = length;
		this.limit = new _Limit(limit.lower, limit.upper);
	}
}

class _FourBarConstraint {
	constructor(
		name,
		type,
		joint,
		ground,
		gound_offset,
		length,
		init_angle
	) {
		this.name = name;
		this.type = type;
		this.joint = joint;
		this.ground = ground;
		this.gound_offset = gound_offset;
		this.length = length;
		this.init_angle = init_angle;
	}
}
class _Actuator {
	constructor(
		name,
		type,
		tube_parent,
		rod_parent,
		tube_offset,
		rod_offset,
		limit,
		tube_visual,
		rod_visual,
	) {
		this.name = name;
		this.type = type;
		this.tube_parent = tube_parent;
		this.rod_parent = rod_parent;
		this.tube_offset = new _Translation(tube_offset[0], tube_offset[1], tube_offset[2]);
		this.rod_offset = new _Translation(rod_offset[0], rod_offset[1], rod_offset[2]);
		this.limit = new _Limit(limit.lower, limit.upper);
		this.tube_visual = tube_visual;
		this.rod_visual = rod_visual;
	}
}
class _Actuator3 {
	constructor(
		name,
		type,
		tube_parent,
		rod_parent,
		tube_offset,
		rod_offset,
		tube_visual,
		mid_visual,
		rod_visual
	) {
		this.name = name;
		this.type = type;
		this.tube_parent = tube_parent;
		this.rod_parent = rod_parent;
		this.tube_offset = tube_offset;
		this.rod_offset = rod_offset;
		this.tube_visual = tube_visual;
		this.mid_visual = mid_visual;
		this.rod_visual = rod_visual;
	}
}

class _Limit {
	constructor(lower, upper) {
		this.lower = lower;
		this.upper = upper;
	}
}

class _Axis {
	constructor(x = 0, y = 0, z = 0) {
		this.x = x;
		this.y = y;
		this.z = z;
	}
}

class _Translation {
	constructor(x = 0, y = 0, z = 0) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	length() {
		return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
	}
}

class _Orientation {
	constructor(x = 0, y = 0, z = 0) {
		this.x = x;
		this.y = y;
		this.z = z;
	}
}

class _Visual {
	constructor(origin_translation, origin_orientation, geometry, material) {
		this.origin_translation = origin_translation;
		this.origin_orientation = origin_orientation;
		this.geometry = geometry;
		this.material = material;
	}
}

class _TopoNode {
	constructor(name = "", parent = null, children = [], model = null) {
		this.name = name;
		this.parent = parent;
		this.children = children;
		this.model = model;
	}

	add(otherNode) {
		otherNode.parent = this;
		this.children.push(otherNode);
	}

	remove(otherNode) {
		const index = this.children.indexOf(otherNode);

		if (index !== - 1) {
			otherNode.parent = null;
			this.children.splice(index, 1);
		}
	}

	traverse(callback) {
		callback(this);

		const children = this.children;
		for (let i = 0, l = children.length; i < l; i++) {

			children[i].traverse(callback);

		}
	}
}
class _TopoJoint extends _TopoNode {
	constructor(name = "", parent = null, children = []) {
		super(name, parent, children);
		this.isJoint = true;
		this.fixed = false;
	}
}

class _TopoLink extends _TopoNode {
	constructor(name = "", parent = null, children = []) {
		super(name, parent, children);
		this.isLink = true;
	}
}
class _TopoFourBarJoint extends _TopoJoint {


	constructor(name = "", parent = null, children = [], path = []) {
		super(name, parent, children);
		this.isFourBarJoint = true;
		this.path = path;
	}
}
class _TopoActuatorJoint extends _TopoJoint {

	constructor(name = "", parent = null, children = []) {
		super(name, parent, children);
		this.isActuatorJoint = true;
	}
}
class _TopoActuatorLink extends _TopoLink {
	constructor(name = "", parent = null, children = [], twin = null) {
		super(name, parent, children);
		this.isActuatorLink = true;
		this.twin = twin;
	}
}
function _buildRobotTopo(robot) {

	let robotModel = robot.robotModel;

	let base_link_model = robotModel.children[0]



	function copy_tree(curr_model) {

		let currTopoNode;
		if (curr_model.isJoint) {
			currTopoNode = new _TopoJoint(curr_model.name);
			currTopoNode.model = curr_model;
			for (let i = 0; i < curr_model.children.length; i++) {
				currTopoNode.add(copy_tree(curr_model.children[i]))
			}
		}
		else if (curr_model.isLink) {
			currTopoNode = new _TopoLink(curr_model.name);
			currTopoNode.model = curr_model;
			for (let i = 0; i < curr_model.children.length; i++) {
				currTopoNode.add(copy_tree(curr_model.children[i]))
			}
		}

		return currTopoNode;
	}

	let robotTopo = copy_tree(base_link_model);

	let robotAdjacencyList = getAdjacencyList(base_link_model);



	robot.constraints.forEach(cst => {

		if (cst.type == "four-bar") {
			let jointOfCst = getChildByName(robotTopo, cst.joint);
			let groundOfCst = getChildByName(robotTopo, cst.ground);
			let drived_rocker = jointOfCst.children[0];


			let four_bar_paths = getTopoPaths(groundOfCst, drived_rocker)

			let fourBarJoint = new _TopoFourBarJoint(cst.name)
			fourBarJoint.path = four_bar_paths[0];
			let floating_link = four_bar_paths[0][4];
			fourBarJoint.add(floating_link);
			let ground_to_output_paths = getTopoPaths(groundOfCst, floating_link);

			groundOfCst.add(fourBarJoint);
			groundOfCst.remove(ground_to_output_paths[0][1]);
			floating_link.remove(jointOfCst)
		}
		else {
			console.error("Unsupported constraint type: ", cst);
		}

	});

	return robotTopo;
}

function _buildActuatorsTopo(robot) {

	let robotTopo = robot.robotTopo;

	let actuatorTopos = {};
	robot.actuators.forEach(actuator => {

		let tempTopo = copy_topo(robotTopo);

		let tube_parent = getChildByName(tempTopo, actuator.tube_parent);
		let rod_parent = getChildByName(tempTopo, actuator.rod_parent);

		robot.actuators.forEach(otherActor => {

			if (otherActor.name == actuator.name)
				return;

			let other_tube_parent = getChildByName(tempTopo, otherActor.tube_parent);
			let other_rod_parent = getChildByName(tempTopo, otherActor.rod_parent);
			let tube2rodPath = getTopoPaths(other_tube_parent, other_rod_parent)
			let rod2tubePath = getTopoPaths(other_rod_parent, other_tube_parent)

			if (tube2rodPath.length > 0) {
				let tube2rodDOF = tube2rodPath[0].length - 2;

				if (tube2rodDOF == 1) {
					tube2rodPath[0][1].fixed = true;
				}
				else if (tube2rodDOF > 1) {
					let otherActorLink = new _TopoLink("link_" + otherActor.name);
					let rodParent2OtherActorLinkJoint = new _TopoJoint("joint_" + other_rod_parent.name + "_" + otherActor.name);
					let otherActorLink2tubeParentJoint = new _TopoJoint("joint_" + other_tube_parent.name + "_" + otherActor.name);
					other_rod_parent.add(rodParent2OtherActorLinkJoint);
					rodParent2OtherActorLinkJoint.add(otherActorLink);
					otherActorLink.add(otherActorLink2tubeParentJoint);
					otherActorLink2tubeParentJoint.add(other_tube_parent);

				}
				else { console.error("Actuator", actuator, "topo error!",); }
			}
			else {

				let tod2tubeDOF = rod2tubePath[0].length - 2;

				if (tod2tubeDOF == 1) {
					rod2tubePath[0][1].fixed = true;
				}
				else if (tod2tubeDOF > 1) {
					let otherActorLink = new _TopoLink(otherActor.name + "_link");
					let tubeParent2OtherActorLinkJoint = new _TopoJoint("joint_" + other_tube_parent.name + "_" + otherActor.name);
					let otherActorLink2RodParentJoint = new _TopoJoint("joint_" + other_rod_parent.name + "_" + otherActor.name);

					other_tube_parent.add(tubeParent2OtherActorLinkJoint);
					tubeParent2OtherActorLinkJoint.add(otherActorLink);
					otherActorLink.add(otherActorLink2RodParentJoint);
					otherActorLink2RodParentJoint.add(other_rod_parent);

				}
				else { console.error("Actuator", actuator, "topo error!",); }
			}
			let pathRod2Tube = getTopoPaths(rod_parent, tube_parent);
			let pathTube2Rod = getTopoPaths(tube_parent, rod_parent);
			let numJointsInPath = 0;
			if (pathRod2Tube.length > 0) {
				pathRod2Tube[0].forEach(ele => {
					if (ele.isJoint && !ele.fixed)
						numJointsInPath++;
				});
			}
			if (pathTube2Rod.length > 0) {
				pathTube2Rod[0].forEach(ele => {
					if (ele.isJoint && !ele.fixed)
						numJointsInPath++;
				});
			}

			actuatorTopos[actuator.name] = {};
			actuatorTopos[actuator.name]["name"] = actuator.name;

			if (numJointsInPath == 1) {
				if (pathRod2Tube.length > 0) {
					actuatorTopos[actuator.name]["topo"] = pathRod2Tube[0];

					pathRod2Tube[0].forEach(ele => {
						if (ele.isJoint && !ele.fixed) {
							actuatorTopos[actuator.name]["driving"] = ele;
							if (ele.isFourBarJoint)
								actuatorTopos[actuator.name]["type"] = "tri_four_bar";
							else
								actuatorTopos[actuator.name]["type"] = "tri";
						}
					})
				}
				else if (pathTube2Rod.length > 0) {
					actuatorTopos[actuator.name]["topo"] = pathTube2Rod[0];

					pathTube2Rod[0].forEach(ele => {
						if (ele.isJoint && !ele.fixed) {
							actuatorTopos[actuator.name]["driving"] = ele;
							if (ele.isFourBarJoint)
								actuatorTopos[actuator.name]["type"] = "tri_four_bar";
							else
								actuatorTopos[actuator.name]["type"] = "tri";
						}
					})
				}
			}
			else if (numJointsInPath == 4) {

				let pathLoop = merge_topo_path(pathRod2Tube[0], pathTube2Rod[0]);

				actuatorTopos[actuator.name]["topo"] = pathLoop;
				actuatorTopos[actuator.name]["driving"] = pathLoop[1];
				actuatorTopos[actuator.name]["type"] = "generalized_four_bar";
			}

		});

		actuatorTopos[actuator.name]["rodJointModel"] = null;
		actuatorTopos[actuator.name]["tubeJointModel"] = null;

	})

	return actuatorTopos;

}
function _updateActuatorsModel(robot) {

	let actuatorsTopo = robot.actuatorsTopo;

	robot.actuators.forEach(actuator => {

		let jointAModel = actuatorsTopo[actuator.name].rodJointModel;
		let jointBModel = actuatorsTopo[actuator.name].tubeJointModel;

		let A_world = new THREE.Vector3();
		jointAModel.getWorldPosition(A_world);
		let B_world = new THREE.Vector3();
		jointBModel.getWorldPosition(B_world);

		jointAModel.rotation.set(0, 0, 0);
		jointBModel.rotation.set(0, 0, 0);

		let B_local_in_A = jointAModel.worldToLocal(B_world);
		let A_local_in_B = jointBModel.worldToLocal(A_world);
		let currAxis = new THREE.Vector3(1, 0, 0);
		let targetAxis = B_local_in_A;

		let rotationAngle = currAxis.angleTo(targetAxis);
		let rotationAxis = new THREE.Vector3();
		rotationAxis.crossVectors(currAxis, targetAxis);
		rotationAxis.normalize();
		jointAModel.rotateOnAxis(rotationAxis, rotationAngle);

		currAxis = new THREE.Vector3(1, 0, 0);
		targetAxis = A_local_in_B;

		rotationAngle = currAxis.angleTo(targetAxis);
		rotationAxis = new THREE.Vector3();
		rotationAxis.crossVectors(currAxis, targetAxis);
		rotationAxis.normalize();
		jointBModel.rotateOnAxis(rotationAxis, rotationAngle);

	});

}

function _actuatorSolver(robot, actuatorName, targetLength) {

	let actuatorsTopo = robot.actuatorsTopo;

	let actuatorTopo = actuatorsTopo[actuatorName];

	let result;
	if (actuatorTopo.type == "A") {

	}
	else if (actuatorTopo.type == "B") {

	}
	else if (actuatorTopo.type == "tri_four_bar") {

		let currDrivingAngle = _getCurrDrivingAngle(robot, actuatorTopo.driving);

		let ObjectFunc = (drivingAngle) => {

			_fourBarSolver(robot, actuatorTopo.driving.name, drivingAngle);

			let delLength = _getWorldDistance(actuatorTopo.rodJointModel, actuatorTopo.tubeJointModel) - targetLength;

			return delLength;
		}
		result = Secant(ObjectFunc, currDrivingAngle);

	}
	else if (actuatorTopo.type == "generalized_four_bar") {

		let base_driving_joint_model = actuatorTopo.driving.path[3].model
		let driving_floating_joint_model = actuatorTopo.topo[3].model;
		let floating_drived_joint_model = actuatorTopo.topo[5].model;
		let constraint_joint_model = actuatorTopo.topo[7].model;

		let driving_len = _getWorldDistance(
			base_driving_joint_model,
			driving_floating_joint_model
		);

		let floating_len = _getWorldDistance(
			driving_floating_joint_model,
			floating_drived_joint_model
		);

		let driven_len = _getWorldDistance(
			floating_drived_joint_model,
			constraint_joint_model,
		);

		let ObjectFunc = (drivingAngle) => {

			_fourBarSolver(robot, actuatorTopo.driving.name, drivingAngle);

			let ground_len = _getWorldDistance(
				base_driving_joint_model,
				constraint_joint_model
			);

			let v2 = new THREE.Vector3();
			constraint_joint_model.getWorldPosition(v2);
			let temp = new THREE.Vector3();
			base_driving_joint_model.getWorldPosition(temp);
			v2.sub(temp);

			let v1 = new THREE.Vector3();
			driving_floating_joint_model.getWorldPosition(v1);
			base_driving_joint_model.getWorldPosition(temp);
			v1.sub(temp);
			let driving_angle = v1.angleTo(v2);

			let fourbarRes = solveFourBar(driving_angle, ground_len, driving_len, floating_len, driven_len);

			driving_floating_joint_model.rotation.z = fourbarRes.driving_floating_angle;
			floating_drived_joint_model.rotation.z = fourbarRes.floating_driven_angle;

			let delLength = _getWorldDistance(actuatorTopo.rodJointModel, actuatorTopo.tubeJointModel) - targetLength;

			return delLength;
		}

		let currDrivingAngle = _getCurrDrivingAngle(robot, actuatorTopo.driving);
		result = NewtonRaphson(ObjectFunc, currDrivingAngle);

	}
	else {
		console.error("Unsupported actuator topology type:", actuatorTopo.type);
	}

	_updateActuatorsModel(robot);

	return result;
}

function _IKSolver(robot, EEModel, targetWorldMatrix) {
	if (EEModel && EEModel.material && EEModel.material.color) {
		EEModel.material.color.set(0xff0000);
	}

	function optOneActuator(actuator) {
		const actuatorName = actuator.name;
		const lower = actuator.limit.lower;
		const upper = actuator.limit.upper;

		const objectFunc = (length) => {
			_actuatorSolver(robot, actuatorName, length);
			return LieAlgebraLoss(targetWorldMatrix, EEModel.matrixWorld);
		};

		return Golden(objectFunc, lower, upper);
	}

	function vectorSubtract(v1, v2) {
		return v1.map((val, i) => val - v2[i]);
	}

	function dotProduct(v1, v2) {
		return v1.reduce((sum, val, i) => sum + val * v2[i], 0);
	}

	const maxIterations = 100;
	const tolerance = 1e-6;
	let nfev = 0;

	let currentLengths = robot.actuators.map((actuator) => {
		const actuatorTopo = robot.actuatorsTopo[actuator.name];
		return _getWorldDistance(actuatorTopo.rodJointModel, actuatorTopo.tubeJointModel);
	});

	for (let iter = 0; iter < maxIterations; iter++) {
		const nextLengths = [];

		for (let i = 0; i < robot.actuators.length; i++) {
			const actuator = robot.actuators[i];
			const res = optOneActuator(actuator);
			nfev += res.nfev;
			nextLengths.push(res.x);
		}

		const step = vectorSubtract(nextLengths, currentLengths);
		const stepNorm = Math.sqrt(dotProduct(step, step));

		if (stepNorm < tolerance) {
			return { solution: nextLengths, iterations: iter + 1, success: true, nfev };
		}

		currentLengths = nextLengths;
	}

	return { solution: currentLengths, iterations: maxIterations, success: false, nfev };

}
function Secant(ObjectFunc, x0, tol = 1e-6, maxiter = 100) {

	let funcCalls = 0;

	let f = ObjectFunc;

	let x_prev = x0;
	let x_curr = x0 + 0.001;

	let f_prev = f(x_prev); funcCalls += 1;
	let f_curr = f(x_curr); funcCalls += 1;

	for (let iter = 0; iter < maxiter; iter++) {
		if (Math.abs(f_curr) < tol) {
			return { nfev: funcCalls };
		}

		if (Math.abs(f_curr - f_prev) < 1e-12) {
			console.error("Division by zero detected in the iteration. Method failed.");
		}

		let x_next = x_curr - f_curr * (x_curr - x_prev) / (f_curr - f_prev);

		x_prev = x_curr;
		x_curr = x_next;
		f_prev = f_curr;
		f_curr = f(x_curr); funcCalls += 1;

	}

}
function NewtonRaphson(ObjectFunc, x0, tol = 1e-6, maxiter = 100) {

	let results = [];

	let f = ObjectFunc;

	let x_curr = x0;
	let f_curr = f(x_curr);

	for (let iter = 0; iter < maxiter; iter++) {

		results.push(f_curr);

		if (Math.abs(f_curr) < tol) {
			return results;
		}

		let h = 1e-3;
		let f_prime = (f(x_curr + h) - f_curr) / (1 * h);
		if (Math.abs(f_prime) < 1e-12) {
			return results;
		}

		x_curr -= f_curr / f_prime;
		f_curr = f(x_curr);
	}

}
function Golden(ObjectFunc, a, b, tol = 1e-3, maxIter = 100) {

	let nfev = 0;

	let f = ObjectFunc;
	const phi = (1 + Math.sqrt(5)) / 2;
	let x1 = b - (b - a) / phi;
	let x2 = a + (b - a) / phi;
	let f1 = f(x1);
	let f2 = f(x2);
	nfev += 2;

	let iter = 0;
	while ((b - a) > tol) {
		iter++;
		if (iter >= maxIter) {
			console.error("Maximum iterations reached. Method did not converge.");
			return null;
		}

		if (f1 < f2) {
			b = x2;
			x2 = x1;
			f2 = f1;
			x1 = b - (b - a) / phi;
			f1 = f(x1);
		} else {
			a = x1;
			x1 = x2;
			f1 = f2;
			x2 = a + (b - a) / phi;
			f2 = f(x2);
		}

		nfev += 1;
	}

	const xStar = (a + b) / 2;
	return { x: xStar, f: f(xStar), nfev: nfev };

}

function LieAlgebraLoss(T1, T2) {

	function logSO3(R) {
		const trace = R.elements[0] + R.elements[5] + R.elements[10];
		let theta = Math.acos(Math.min(Math.max((trace - 1) / 2, -1), 1));

		if (theta < 1e-5) {
			return new THREE.Vector3(0, 0, 0);
		}

		const wx = (R.elements[9] - R.elements[6]) / (2 * Math.sin(theta));
		const wy = (R.elements[2] - R.elements[8]) / (2 * Math.sin(theta));
		const wz = (R.elements[4] - R.elements[1]) / (2 * Math.sin(theta));
		return new THREE.Vector3(wx, wy, wz).multiplyScalar(theta);
	}
	let T1_inv = new THREE.Matrix4().copy(T1).invert();
	let dT = new THREE.Matrix4().multiplyMatrices(T1_inv, T2);
	let R = new THREE.Matrix4().copy(dT);
	R.elements[12] = R.elements[13] = R.elements[14] = 0;

	let t = new THREE.Vector3().setFromMatrixPosition(dT);
	let omega = logSO3(R);
	let xi = new THREE.Vector3().copy(omega).toArray().concat(t.toArray());
	let norm = Math.sqrt(xi.reduce((sum, v) => sum + v * v, 0));
	return norm;
}

function _getWorldDistance(modelA, modelB) {

	let A_world = new THREE.Vector3();
	modelA.getWorldPosition(A_world);
	let B_world = new THREE.Vector3();
	modelB.getWorldPosition(B_world);

	return A_world.distanceTo(B_world);

}

function _getCurrDrivingAngle(robot, fourBarConstraint) {

	let robotTopo = robot.robotTopo;
	let fourbarTopo = getChildByName(robotTopo, fourBarConstraint.name);
	let path = fourbarTopo.path;
	let ground_driving_joint_model = path[1].model;
	let drivingAngle = ground_driving_joint_model.rotation.z;

	return drivingAngle;

}

function _fourBarSolver(robot, fourBarConstraintName, drivingAngle) {

	let robotTopo = robot.robotTopo;

	let fourBarConstraint = robot.constraintMap.get(fourBarConstraintName);

	let ground_offset = fourBarConstraint.gound_offset;

	let fourbarTopo = getChildByName(robotTopo, fourBarConstraint.name);
	let path = fourbarTopo.path;

	let ground_driving_joint_model = path[1].model;
	let driving_floating_joint_model = path[3].model;
	let floating_drived_joint_model = path[5].model;

	ground_driving_joint_model.rotation.z = drivingAngle;

	let ground_len = getEuclideanLength([
		ground_offset[0] - ground_driving_joint_model.position.x,
		ground_offset[1] - ground_driving_joint_model.position.y,
		ground_offset[2] - ground_driving_joint_model.position.z
	])
	let driving_len = driving_floating_joint_model.position.length();
	let floating_len = floating_drived_joint_model.position.length();
	let driven_len = fourBarConstraint.length;

	let fourbarRes = solveFourBar(Math.PI - drivingAngle - Math.atan(Math.abs(ground_offset[1]) / Math.abs(ground_offset[0])), ground_len, driving_len, floating_len, driven_len);

	driving_floating_joint_model.rotation.z = fourbarRes.driving_floating_angle;
	floating_drived_joint_model.rotation.z = fourbarRes.floating_driven_angle;

}

function clone_topo_node(otherNode) {
	let newTopoNode;
	if (otherNode.isJoint) {
		if (otherNode.isFourBarJoint) {

			newTopoNode = new _TopoFourBarJoint(otherNode.name);
			newTopoNode.path = otherNode.path;
		}
		else {
			newTopoNode = new _TopoJoint(otherNode.name);
		}

	}
	else if (otherNode.isLink) {
		newTopoNode = new _TopoLink(otherNode.name);
	}

	newTopoNode.fixed = otherNode.fixed;
	newTopoNode.model = otherNode.model;

	return newTopoNode;
}

function copy_topo(currTopoNode) {
	let newTopoNode = clone_topo_node(currTopoNode);
	for (let i = 0; i < currTopoNode.children.length; i++) {
		newTopoNode.add(copy_topo(currTopoNode.children[i]));
	}

	return newTopoNode;
}

function merge_topo_path(path1, path2) {

	if (path1[0].name != path2[path2.length - 1].name
		|| path2[0].name != path1[path1.length - 1].name) { console.error("Merge Error!"); }
	let tempLoop = [];
	for (let i = 0; i < path1.length; i++) { tempLoop.push(clone_topo_node(path1[i])); }
	for (let i = 1; i < path2.length - 1; i++) { tempLoop.push(clone_topo_node(path2[i])); }

	let loop = [];
	let drivingIndex = 0;

	for (; drivingIndex < tempLoop.length; drivingIndex++) {
		if (tempLoop[drivingIndex].isFourBarJoint) { break; }
	}

	if (drivingIndex > 0) {
		loop.push(clone_topo_node(tempLoop[drivingIndex - 1]));
		for (let i = drivingIndex; i < tempLoop.length; i++) { loop.push(clone_topo_node(tempLoop[i])); }
		for (let i = 0; i < drivingIndex; i++) { loop.push(clone_topo_node(tempLoop[i])); }
	}
	else {
		loop.push(clone_topo_node(tempLoop[tempLoop.length - 1]));
		for (let i = 0; i < tempLoop.length; i++) { loop.push(clone_topo_node(tempLoop[i])); }
	}

	return loop;
}

function getTopoPaths(fromTopoNode, toTopoNode) {

	function dfs_all_paths(from, to, path = []) {

		path = path.concat(from);

		if (from === to) {
			return [path];
		}

		let paths = []

		for (let i = 0; i < from.children.length; i++) {
			let currChild = from.children[i];
			if (!path.includes(currChild)) {

				let new_paths = dfs_all_paths(currChild, to, path);

				for (let j = 0; j < new_paths.length; j++) {
					paths.push(new_paths[j]);
				}
			}
		}

		return paths;
	}

	let paths = dfs_all_paths(fromTopoNode, toTopoNode);

	return paths;
}

function getAdjacencyList(rootObject) {
	let adjacencyList = {};
	rootObject.traverse(currNode => {
		if (!adjacencyList[currNode.name]) {
			adjacencyList[currNode.name] = [];
		}

		currNode.children.forEach(child => {
			adjacencyList[currNode.name].push(child.name);
		});
	});
	return adjacencyList;
}
function getChildByName(object, childName) {

	let child = null;

	object.traverse(currChild => {
		if (currChild.name == childName) {

			if (child == null) {
				child = currChild;
			}
			else {
				console.error("Duplicate node name:", childName);
			}
		}
	});

	return child;
}

function getEuclideanLength(vec) {
	let sum = 0.0;
	vec.forEach(e => {

		sum += e * e;
	});

	return Math.sqrt(sum);
}

function solveFourBar(driving_angle, ground_len, driving_len, floating_len, drived_len) {
	let a = driving_angle;

	const diag_len = Math.sqrt(ground_len * ground_len + driving_len * driving_len - 2 * ground_len * driving_len * Math.cos(a));

	const sinA = Math.sin(a);
	const sinB = (ground_len * sinA) / diag_len;
	const b = Math.asin(sinB);

	const d = Math.acos((floating_len * floating_len + drived_len * drived_len - diag_len * diag_len) / (2 * floating_len * drived_len));
	const bb = Math.acos((floating_len * floating_len + diag_len * diag_len - drived_len * drived_len) / (2 * floating_len * diag_len));

	let result = {
		driving_floating_angle: Math.PI - b - bb,
		floating_driven_angle: Math.PI - d,
	};

	return result;

}

export {
	_Robot,
	_Link,
	_Joint,
	_Constraint,
	_FourBarConstraint,
	_Actuator, _Actuator3,
	_Limit,
	_Axis,
	_Translation,
	_Orientation,
	_Visual,

	_TopoNode,
	_TopoJoint,
	_TopoLink,

	getChildByName,
	_buildRobotTopo,
	_buildActuatorsTopo,
	_updateActuatorsModel,
	_actuatorSolver,
	_IKSolver,
	_fourBarSolver,

	_getWorldDistance,

	getEuclideanLength,
	solveFourBar
}

