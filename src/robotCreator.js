import * as THREE from 'three';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import {
	_Link,
	_Translation,
	_Orientation,
	getChildByName,
	_buildRobotTopo,
	_buildActuatorsTopo,
	_updateActuatorsModel,
	_fourBarSolver
} from './robotCommon.js';

// import {robotMainTreeCreator} from './robotTopologyBuilder.js'

const objloader = new OBJLoader();


function robotCreator(robot) {

	// robotModel as the container of a robot.
	let robotModel = new THREE.Group();
	robotModel.name = robot.name;

	// base_link as root of the topology tree of a robot.
	let base_link = new THREE.Group();
	base_link.name = "base_link";
	base_link.isLink = true;
	robotModel.add(base_link);

	/**
	 * STEP 1.
	 * Build the topology tree. 
	 * Each link in the topology tree is an empty mesh with link name.
	 */

	let linkModels = [];

	robot.joints.forEach(joint => {

		let jointModel = createJointModel(joint);

		// 1. parent link
		let parentLinkInRobot = getChildByName(robotModel, joint.parent);

		if (parentLinkInRobot == null) {

			let parentLinkModel = new THREE.Mesh();

			parentLinkModel.isLink = true;

			parentLinkModel.name = joint.parent;

			parentLinkModel.add(jointModel);

			robotModel.add(parentLinkModel);

			linkModels.push(parentLinkModel);
		}
		else {
			parentLinkInRobot.add(jointModel);
		}


		// 2. child link
		let childLinkInRobot = getChildByName(robotModel, joint.child);
		if (childLinkInRobot == null) {

			let childLinkModel = new THREE.Mesh();
			childLinkModel.isLink = true;

			childLinkModel.name = joint.child;

			jointModel.add(childLinkModel);

			linkModels.push(childLinkModel);
		}
		else {
			console.error("Closed loop detected at child link:", joint.child);
		}

	});


	/**
	 * STEP 2.
	 * Create (basic shapes: box, cylinder ...) or load (external meshes: .obj file) 
	 * visual meshes of links in topology tree.
	 */

	// async: Treat both the basic shapes and the external models as promises.
	linkModels.forEach(async linkModel => {

		let linkName = linkModel.name;
		let link = robot.linkMap.get(linkName);

		let LinkModelPromise = createLinkModel(link);

		let result = await LinkModelPromise;

		let visual = link.visual;

		// visual offset
		let visual_origin_translation = visual.origin_translation;
		let visual_origin_orientation = visual.origin_orientation;

		linkModel.geometry = result.geometry.clone();
		linkModel.material = result.material.clone();

		// Euler angle order : X -> Y -> Z
		linkModel.geometry.rotateX(visual_origin_orientation[0]);
		linkModel.geometry.rotateY(visual_origin_orientation[1]);
		linkModel.geometry.rotateZ(visual_origin_orientation[2]);
		linkModel.geometry.translate(visual_origin_translation[0], visual_origin_translation[1], visual_origin_translation[2]);

		robotModel.updateWorldMatrix(true, true);

	});

	robot.robotModel = robotModel;

	/**
	 * topology (directed graph) of robot
	 * tree + constraints + actuators
	 */

	let robotTopo = _buildRobotTopo(robot);
	robot.robotTopo = robotTopo;
	// console.log("topo", robotTopo);

	let actuatorsTopo = _buildActuatorsTopo(robot);
	robot.actuatorsTopo = actuatorsTopo;
	// console.log("actuatorTopo", actuatorsTopo);


	robot.actuators.forEach(actuator => {

		// 1. create tube joint model
		let tubeParentLink = getChildByName(robotModel, actuator.tube_parent);

		let tubeParentLinkJointModel;
		if (actuator.tube_parent === "base_link") {
			let rodParentLink = getChildByName(robotModel, actuator.rod_parent);
			let rodParentLinkJointModel = rodParentLink.parent;
			tubeParentLinkJointModel = rodParentLinkJointModel;
		}
		else {
			tubeParentLinkJointModel = tubeParentLink.parent;
		}

		let tubeJointModel = new THREE.Mesh();
		tubeJointModel.geometry = tubeParentLinkJointModel.geometry.clone();
		tubeJointModel.material = tubeParentLinkJointModel.material.clone();
		tubeJointModel.material.color = new THREE.Color(0x00aa00);

		tubeJointModel.position.set(actuator.tube_offset.x, actuator.tube_offset.y, actuator.tube_offset.z);
		// tubeJointModel.add(tubeLinkModel);

		tubeJointModel.name = actuator.name + "_tubeJointModel";

		tubeParentLink.add(tubeJointModel);


		// 2. create rod joint model
		let rodParentLink = getChildByName(robotModel, actuator.rod_parent);
		let rodParentLinkJointModel;
		if (actuator.rod_parent === "base_link") {
			let tubeParentLink = getChildByName(robotModel, actuator.tube_parent);
			let tubeParentLinkJointModel = tubeParentLink.parent;
			rodParentLinkJointModel = tubeParentLinkJointModel;
		}
		else {
			rodParentLinkJointModel = rodParentLink.parent;
		}

		let rodJointModel = new THREE.Mesh();
		rodJointModel.geometry = rodParentLinkJointModel.geometry.clone();
		rodJointModel.material = rodParentLinkJointModel.material.clone();
		rodJointModel.material.color = new THREE.Color(0x00aa00);

		rodJointModel.position.set(actuator.rod_offset.x, actuator.rod_offset.y, actuator.rod_offset.z);
		// rodJointModel.add(rodLinkModel);

		rodJointModel.name = actuator.name + "_rodJointModel";

		rodParentLink.add(rodJointModel);


		actuatorsTopo[actuator.name].rodJointModel = rodJointModel;
		actuatorsTopo[actuator.name].tubeJointModel = tubeJointModel;


		// console.log(actuatorsTopo);
		for (const key in actuatorsTopo) {
			if (Object.prototype.hasOwnProperty.call(actuatorsTopo, key)) {
				const actuatorTopo = actuatorsTopo[key];

				actuatorTopo.topo.forEach(element => {
					if (element.name == "joint_" + rodParentLink.name + "_" + actuator.name) {
						element.model = rodJointModel;
					}
					if (element.name == "joint_" + tubeParentLink.name + "_" + actuator.name) {
						element.model = tubeJointModel;
					}
				});

			}
		}

	});


	/**
	 * create tube and rod link models for each actuator
	 */
	robot.actuators.forEach(async actuator => {

		/**
		 * create tube link Model
		 */
		let tubeLink = new _Link(
			actuator.name + "_tube_model",
			new _Translation(),
			new _Orientation(),
			actuator.tube_visual
		);
		let tubeLinkModelPromise = createLinkModel(tubeLink);
		let tubeLinkResult = await tubeLinkModelPromise;

		// visual offset (pass)

		let tubeLinkModel = new THREE.Mesh();
		tubeLinkModel.isActuatorModel = true; // tag, curr _tube to find rod, getChildByName(robotModel, this.name _rod)
		tubeLinkModel.name = actuator.name + "_tube";
		tubeLinkModel.geometry = tubeLinkResult.geometry.clone();
		tubeLinkModel.material = tubeLinkResult.material.clone();
		tubeLinkModel.material.transparent = true;
		tubeLinkModel.material.opacity = 0.4;

		getChildByName(robotModel, actuator.name + "_tubeJointModel").add(tubeLinkModel);


		/**
		 * create rod link Model
		 */
		let rodLink = new _Link(
			actuator.name + "_rod_model",
			new _Translation(),
			new _Orientation(),
			actuator.rod_visual
		);
		let rodLinkModelPromise = createLinkModel(rodLink);
		let rodLinkResult = await rodLinkModelPromise;

		// visual offset (pass)

		let rodLinkModel = new THREE.Mesh();
		rodLinkModel.isActuatorModel = true; // tag
		rodLinkModel.name = actuator.name + "_rod";
		rodLinkModel.geometry = rodLinkResult.geometry.clone();
		rodLinkModel.material = rodLinkResult.material.clone();

		getChildByName(robotModel, actuator.name + "_rodJointModel").add(rodLinkModel);

	});


	/**
	 * constraints init
	 */
	// init jointL_base_angle
	robot.constraints.forEach(constraint => {

		if (constraint.type === "four-bar") {

			// _fourBarSolver(robot, constraint.name, constraint.init_angle);
			// _fourBarSolver(robot, constraint.name, 1.5);
			// _fourBarSolver(robot, constraint.name, 0.9);
			_fourBarSolver(robot, constraint.name, 1.2);

		}
		else if (constraint.type === "triangle-prismatic") {

			// let jointA = robot.jointMap.get(constraint.jointA);
			let jointAModel = getChildByName(robotModel, constraint.jointA);

			// let jointB = robot.jointMap.get(constraint.jointB);
			let jointBModel = getChildByName(robotModel, constraint.jointB);

			// let jointL = robot.jointMap.get(constraint.jointL);
			let jointLModel = getChildByName(robotModel, constraint.jointL);


			let A_world = new THREE.Vector3();
			jointAModel.getWorldPosition(A_world);
			// console.log(A_world);

			let B_world = new THREE.Vector3();
			jointBModel.getWorldPosition(B_world);

			let L_world = new THREE.Vector3();
			jointLModel.getWorldPosition(L_world);

			let LB = new THREE.Vector3();
			LB.subVectors(B_world, L_world);
			let LA = new THREE.Vector3();
			LA.subVectors(A_world, L_world);

			constraint.jointL_base_angle = LA.angleTo(LB);

			console.log(constraint.name);
			console.log(constraint.jointL_base_angle);
		}


	});

	// init actuators models (rod and tube lookat each other)
	_updateActuatorsModel(robot);


	// _actuatorSolver test
	// _actuatorSolver(robotTopo, actuatorsTopo, "hydraulic_cylinder_1", 6.0);
	// _actuatorSolver(robot, "hydraulic_cylinder_1", 6.0);

	// _actuatorSolver(robot, "hydraulic_cylinder_2", 1.0);
	// _actuatorSolver(robot, "hydraulic_cylinder_1", 5.0);
	// _actuatorSolver(robot, "hydraulic_cylinder_2", 1.5);
	// _actuatorSolver(robot, "hydraulic_cylinder_1", 4.0);

	// inverse kinematics
	// target T ∈ se2 se3







	/**
	 * actuators init
	 */
	robot.actuators.forEach(actuator => {

		// console.log(actuator);

		// rod and tube look at each other

		// a function   actuatorHandler(actuator)






	});


	/**
	 * actuators handle
	 */
	robot.actuators.forEach(actuator => {

		// build actuators topos



	})

	// robotConstraintHandler(robot, robotModel, null);

	// return robotModel;
	return robot;

}




/**
 * only for online test
 */
function robotCreatorWithObjText(robot, objTexts) {

	let robotModel = new THREE.Group(); // base_link
	robotModel.name = robot.name;

	let base_link = new THREE.Group();
	base_link.name = "base_link";
	robotModel.add(base_link);

	// fixedJointModel.position.set();

	/**
	 * 1. build topology tree
	 */

	let linkModels = [];

	robot.joints.forEach(joint => {


		let jointModel = createJointModel(joint);

		// 1. parent link
		let parentLinkInRobot = getChildByName(robotModel, joint.parent);

		if (parentLinkInRobot == null) {

			let parentLinkModel = new THREE.Mesh();

			parentLinkModel.isLink = true;

			parentLinkModel.name = joint.parent;

			parentLinkModel.add(jointModel);

			robotModel.add(parentLinkModel);

			linkModels.push(parentLinkModel);
		}
		else {
			parentLinkInRobot.add(jointModel);
		}


		// 2. child link
		let childLinkInRobot = getChildByName(robotModel, joint.child);
		if (childLinkInRobot == null) {

			let childLinkModel = new THREE.Mesh();
			childLinkModel.isLink = true;

			childLinkModel.name = joint.child;

			jointModel.add(childLinkModel);

			linkModels.push(childLinkModel);
		}
		else {
			console.error("Closed loop detected at child link:", joint.child);
		}

	});



	linkModels.forEach(async linkModel => {

		let linkName = linkModel.name;
		let link = robot.linkMap.get(linkName);
		let visual = link.visual;

		let result;

		if (link.visual.geometry.type != 'mesh') {
			let LinkModelPromise = createLinkModel(link);
			result = await LinkModelPromise;
		}
		else {
			let meshURL = visual.geometry.url;
			let meshObjText = objTexts[meshURL];

			let object = objloader.parse(meshObjText);
			object.children[0].name = link.name;


			let color = new THREE.Color(visual.material.color[0] / 255, visual.material.color[1] / 255, visual.material.color[2] / 255);
			const material = new THREE.MeshPhongMaterial();
			material.color = color;

			object.children[0].material = material;

			if (visual.geometry.scale != null) {
				let meshScale = visual.geometry.scale;
				object.children[0].geometry.scale(meshScale[0], meshScale[1], meshScale[2]);
			}

			object.children[0].isLink = true;

			result = object.children[0];

		}


		// visual offset
		let visual_origin_translation = visual.origin_translation;
		let visual_origin_orientation = visual.origin_orientation;

		// object.children[0].geometry.rotateX(-1.57);

		linkModel.geometry = result.geometry.clone();
		linkModel.material = result.material.clone();

		// Euler angle : X -> Y -> Z
		linkModel.geometry.rotateX(visual_origin_orientation[0]);
		linkModel.geometry.rotateY(visual_origin_orientation[1]);
		linkModel.geometry.rotateZ(visual_origin_orientation[2]);
		linkModel.geometry.translate(visual_origin_translation[0], visual_origin_translation[1], visual_origin_translation[2]);

		// linkModel.rotation.set(visual_origin_orientation[0], visual_origin_orientation[1], visual_origin_orientation[2]);
		// linkModel.position.set(visual_origin_translation[0], visual_origin_translation[1], visual_origin_translation[2]);

		// console.log(linkModel);

		robotModel.updateWorldMatrix(true, true);

		// robotModel.updateMatrixWorld(true);


	});


	/**
	 * constraints init
	 */
	// init jointL_base_angle
	robot.constraints.forEach(constraint => {

		// let jointA = robot.jointMap.get(constraint.jointA);
		let jointAModel = getChildByName(robotModel, constraint.jointA);

		// let jointB = robot.jointMap.get(constraint.jointB);
		let jointBModel = getChildByName(robotModel, constraint.jointB);

		// let jointL = robot.jointMap.get(constraint.jointL);
		let jointLModel = getChildByName(robotModel, constraint.jointL);


		let A_world = new THREE.Vector3();
		jointAModel.getWorldPosition(A_world);
		// console.log(A_world);

		let B_world = new THREE.Vector3();
		jointBModel.getWorldPosition(B_world);

		let L_world = new THREE.Vector3();
		jointLModel.getWorldPosition(L_world);

		let LB = new THREE.Vector3();
		LB.subVectors(B_world, L_world);
		let LA = new THREE.Vector3();
		LA.subVectors(A_world, L_world);

		constraint.jointL_base_angle = LA.angleTo(LB);

		console.log(constraint.name);
		console.log(constraint.jointL_base_angle);

	});

	// robotConstraintHandler(robot, robotModel, null);

	return robotModel;

}




function getTopologyPath(from, end) {
	// dfs

	let result = new _TopoPathInfo(
		// false
		// []
	);

	return result;
}


function createJointModel(joint) {

	if (joint.type == "fixed") {

		let fixedJointModel = new THREE.Object3D();
		fixedJointModel.name = joint.name;

		fixedJointModel.rotation.set(joint.origin_orientation.x, joint.origin_orientation.y, joint.origin_orientation.z);
		fixedJointModel.position.set(joint.origin_translation.x, joint.origin_translation.y, joint.origin_translation.z);

		fixedJointModel.isJoint = true;

		return fixedJointModel;

	}

	else if (joint.type == "revolute") {

		const geometry = new THREE.CylinderGeometry(.2, .2, .51, 16);
		// const geometry = new THREE.CylinderGeometry(.0002, .0002, .0005, 16);
		geometry.rotateZ(Math.PI / 2); // axis to [1, 0, 0]

		let currAxis = new THREE.Vector3(1, 0, 0);
		let targetAxis = new THREE.Vector3(joint.axis.x, joint.axis.y, joint.axis.z);

		// angle between [1, 0, 0] and targetAxis
		let rotationAngle = currAxis.angleTo(targetAxis);
		// rotation axis from currAxis to targetAxis
		let rotationAxis = new THREE.Vector3();
		rotationAxis.crossVectors(currAxis, targetAxis);
		rotationAxis.normalize();
		// console.log(rotationAngle);
		// console.log(rotationAxis);

		let mat4 = new THREE.Matrix4();
		mat4.makeRotationAxis(rotationAxis, rotationAngle);

		geometry.applyMatrix4(mat4);
		const material = new THREE.MeshPhongMaterial({ color: 0xeeeeee, transparent: true, opacity: 0.7 });
		const cylinder = new THREE.Mesh(geometry, material);
		cylinder.castShadow = true;

		// console.log(joint);

		cylinder.rotation.set(joint.origin_orientation.x, joint.origin_orientation.y, joint.origin_orientation.z);
		cylinder.position.set(joint.origin_translation.x, joint.origin_translation.y, joint.origin_translation.z);


		// cylinder.setRotationFromEuler(new THREE.Euler(joint.axis.x, joint.axis.y, joint.axis.z));

		cylinder.name = joint.name;

		cylinder.isJoint = true;

		return cylinder;
	}

	else if (joint.type === "prismatic") {

		// const geometry = new THREE.CylinderGeometry(.2, .2, .5, 16);
		const geometry = new THREE.CylinderGeometry(.0002, .0002, .0005, 16);
		geometry.rotateZ(Math.PI / 2); // axis to [1, 0, 0]

		let currAxis = new THREE.Vector3(1, 0, 0);
		let targetAxis = new THREE.Vector3(joint.axis.x, joint.axis.y, joint.axis.z);

		// angle between [1, 0, 0] and targetAxis
		let rotationAngle = currAxis.angleTo(targetAxis);
		// rotation axis from currAxis to targetAxis
		let rotationAxis = new THREE.Vector3();
		rotationAxis.crossVectors(currAxis, targetAxis);
		rotationAxis.normalize();
		// console.log(rotationAngle);
		// console.log(rotationAxis);

		let mat4 = new THREE.Matrix4();
		mat4.makeRotationAxis(rotationAxis, rotationAngle);

		geometry.applyMatrix4(mat4);
		const material = new THREE.MeshPhongMaterial({ color: 0xeeeeee, transparent: true, opacity: 0. });
		const cylinder = new THREE.Mesh(geometry, material);
		cylinder.castShadow = true;

		// console.log(joint);

		cylinder.rotation.set(joint.origin_orientation.x, joint.origin_orientation.y, joint.origin_orientation.z);
		cylinder.position.set(joint.origin_translation.x, joint.origin_translation.y, joint.origin_translation.z);


		// cylinder.setRotationFromEuler(new THREE.Euler(joint.axis.x, joint.axis.y, joint.axis.z));

		cylinder.name = joint.name;

		cylinder.isJoint = true;

		return cylinder;

	}

	else {
		// todo
		const geometry = new THREE.CylinderGeometry(.2, .2, .5, 16);
		const material = new THREE.MeshPhongMaterial({ color: 0xeeeeee });
		const cylinder = new THREE.Mesh(geometry, material);
		cylinder.rotateZ(Math.PI / 2);
		cylinder.castShadow = true;

		cylinder.isJoint = true;

		return cylinder;
	}

}

// Create both the basic shapes and the external models as promises.
async function createLinkModel(link) {

	let visual = link.visual;

	let geometry_type = visual.geometry.type;

	if (geometry_type == "box") {
		let x = visual.geometry.x;
		let y = visual.geometry.y;
		let z = visual.geometry.z;

		// material
		let color = new THREE.Color(visual.material.color[0] / 255, visual.material.color[1] / 255, visual.material.color[2] / 255);
		const material = new THREE.MeshPhongMaterial({ transparent: true, opacity: 0.5 });
		material.color = color;
		// if (visual.material.opacity) {
		// 	material.transparent = true;
		// 	// material.opacity = visual.material.opacity;
		// 	material.opacity = 0.5;
		// }

		const geometry = new THREE.BoxGeometry(x, y, z);
		geometry.applyMatrix4(new THREE.Matrix4().makeTranslation(x / 2, 0, 0));

		const box = new THREE.Mesh(geometry, material);
		box.castShadow = true;

		box.name = link.name;

		box.isLink = true;

		return box;
	}
	else if (geometry_type == "cylinder") {

		let r1 = visual.geometry.r1;
		let r2 = visual.geometry.r2;
		let h = visual.geometry.h;

		const geometry = new THREE.CylinderGeometry(r1, r2, h, 16);
		geometry.rotateZ(Math.PI / 2); // axis to [1, 0, 0]

		geometry.applyMatrix4(new THREE.Matrix4().makeTranslation(h / 2, 0, 0));

		// material
		let color = new THREE.Color(visual.material.color[0] / 255, visual.material.color[1] / 255, visual.material.color[2] / 255);
		const material = new THREE.MeshPhongMaterial();
		material.color = color;
		if (visual.material.opacity) {
			material.transparent = true;
			material.opacity = visual.material.opacity;
		}

		const cylinder = new THREE.Mesh(geometry, material);

		cylinder.name = link.name;

		cylinder.isLink = true;

		return cylinder;

	}
	else if (geometry_type == "mesh") {

		let meshURL = visual.geometry.url;

		// Check whether the url exists 
		const response = await fetch(meshURL, { method: 'HEAD' });

		if (response.ok) {


			let [object] = await Promise.all([objloader.loadAsync(meshURL)]);
			object.children[0].name = link.name;

			let color = new THREE.Color(visual.material.color[0] / 255, visual.material.color[1] / 255, visual.material.color[2] / 255);
			const material = new THREE.MeshPhongMaterial();
			material.color = color;

			object.children[0].material = material;

			if (visual.geometry.scale != null) {
				let meshScale = visual.geometry.scale;
				object.children[0].geometry.scale(meshScale[0], meshScale[1], meshScale[2]);
			}

			object.children[0].isLink = true;

			return object.children[0];

		} // if (response.ok) 

	}
	else {
		// console.error("不支持的关节几何类型：" + geometry_type);
	}

}



export { robotCreator, robotCreatorWithObjText }
