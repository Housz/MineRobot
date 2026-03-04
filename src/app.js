import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

import { robotParser } from './robotParser.js';
import { robotCreator } from './robotCreator.js';
import { robotActuatorGUICreator } from './robotActuatorGUICreator.js';

class MineVisApp {
	constructor({ canvas } = {}) {
		this.canvas = canvas || document.querySelector('#c');
		if (!this.canvas) {
			throw new Error('Canvas element #c is required.');
		}

		this.renderer = new THREE.WebGLRenderer({ antialias: true, canvas: this.canvas });
		this.renderer.shadowMap.enabled = true;

		this.camera = new THREE.PerspectiveCamera(75, 2, 0.01, 500);
		this.camera.position.set(0, 5, 10);

		this.controls = new OrbitControls(this.camera, this.renderer.domElement);

		this.scene = new THREE.Scene();
		this.scene.background = new THREE.Color(0xaaccff);
		this.scene.fog = new THREE.Fog(0xa0a0a0, 10, 200);

		this._buildEnvironment();

		this.robot = null;
		this.robotGui = null;

		this._render = this._render.bind(this);
		requestAnimationFrame(this._render);
	}

	_buildEnvironment() {
		this.scene.add(new THREE.AmbientLight(0xdddddd, 3));

		const dLight = new THREE.DirectionalLight(0xeeeeee, 3);
		dLight.position.set(5, 2, 4);
		this.scene.add(dLight);

		const ground = new THREE.Mesh(
			new THREE.PlaneGeometry(200, 200),
			new THREE.MeshPhongMaterial({ color: 0xcccccc, depthWrite: false }),
		);
		ground.rotation.x = -Math.PI / 2;
		ground.receiveShadow = true;
		this.scene.add(ground);

		const grid = new THREE.GridHelper(200, 200);
		grid.material.opacity = 0.8;
		grid.material.transparent = true;
		this.scene.add(grid);
	}

	_clearSceneState() {
		if (this.robot && this.robot.robotModel) {
			this.scene.remove(this.robot.robotModel);
		}

		if (this.robotGui) {
			this.robotGui.destroy();
			this.robotGui = null;
		}

		const overlays = [];
		this.scene.traverse((node) => {
			if (node.isMineRobotOverlay) {
				overlays.push(node);
			}
		});

		overlays.forEach((node) => {
			if (node.parent) {
				node.parent.remove(node);
			}
		});
	}

	_addEndEffectorAxes(robot) {
		const endEffectorParentModel =
			robot?.robotModel?.children?.[0]?.children?.[0]?.children?.[0]?.children?.[0]?.children?.[0]?.children?.[1] ?? null;
		if (!endEffectorParentModel) {
			return;
		}

		const axesName = '__mine_robot_ee_axes__';
		const existedAxes = endEffectorParentModel.children.find((child) => child.name === axesName);
		if (existedAxes) {
			endEffectorParentModel.remove(existedAxes);
		}

		const axesHelper = new THREE.AxesHelper(1);
		axesHelper.name = axesName;
		endEffectorParentModel.add(axesHelper);
	}

	loadRobotFromJson(robotJson) {
		this._clearSceneState();

		const robot = robotParser(robotJson);
		this.robot = robotCreator(robot);
		this.scene.add(this.robot.robotModel);

		this.robotGui = robotActuatorGUICreator(this.robot, this.scene);
		this._addEndEffectorAxes(this.robot);

		return this.robot;
	}

	async loadRobotFromUrl(url) {
		const response = await fetch(url);
		if (!response.ok) {
			throw new Error(`Failed to load robot json from ${url}`);
		}

		const robotJson = await response.json();
		return this.loadRobotFromJson(robotJson);
	}

	_resizeRendererToDisplaySize() {
		const canvas = this.renderer.domElement;
		const pixelRatio = window.devicePixelRatio;
		const width = (canvas.clientWidth * pixelRatio) | 0;
		const height = (canvas.clientHeight * pixelRatio) | 0;
		const needResize = canvas.width !== width || canvas.height !== height;
		if (needResize) {
			this.renderer.setSize(width, height, false);
		}

		return needResize;
	}

	_render() {
		if (this._resizeRendererToDisplaySize()) {
			const canvas = this.renderer.domElement;
			this.camera.aspect = canvas.clientWidth / canvas.clientHeight;
			this.camera.updateProjectionMatrix();
		}

		this.renderer.render(this.scene, this.camera);
		requestAnimationFrame(this._render);
	}
}

function createMineVisApp(options = {}) {
	return new MineVisApp(options);
}

export { MineVisApp, createMineVisApp };
