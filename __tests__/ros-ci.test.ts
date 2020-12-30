import * as core from "@actions/core";
import * as io from "@actions/io";
import * as fs from "fs";
import * as path from "path";
import * as yaml from "js-yaml";
import * as actionRosCi from "../src/action-ros-ci";
import * as dep from "../src/dependencies";
import { execBashCommand } from "../src/action-ros-ci";

jest.setTimeout(20000); // in milliseconds

describe("execBashCommand test suite", () => {
	it("calls coreGroup", async () => {
		const mockGroup = jest.spyOn(core, "group");
		const result = await execBashCommand('echo "Hello World"');
		expect(mockGroup).toBeCalled();
		expect(result).toEqual(0);
	});
	it("uses a prefix", async () => {
		const mockGroup = jest.spyOn(core, "group");
		const result = await execBashCommand("Hello World", "echo ");
		expect(mockGroup).toBeCalled();
		expect(result).toEqual(0);
	});
	it("ignores return code", async () => {
		const mockGroup = jest.spyOn(core, "group");
		const options = {
			ignoreReturnCode: true,
		};
		const result = execBashCommand("somebadcommand", "", options);
		expect(mockGroup).toBeCalled();
		expect(result).not.toEqual(0);
	});
});

describe("validate distribution test", () => {
	it("validates that the ros distribution validator acts correctly", async () => {
		expect(actionRosCi.validateDistros("kinetic", "")).toBe(true);
		expect(actionRosCi.validateDistros("melodic", "dashing")).toBe(true);
		expect(actionRosCi.validateDistros("", "eloquent")).toBe(true);
		expect(actionRosCi.validateDistros("", "")).toBe(false);
		expect(actionRosCi.validateDistros("groovy", "")).toBe(false);
		expect(actionRosCi.validateDistros("", "bouncy")).toBe(false);
		expect(actionRosCi.validateDistros("apples", "bananas")).toBe(false);
	});
});

describe("PR dependency management", () => {
	const reposFilesDir = path.join(__dirname, "pr-dependencies");
	const reposFilesTestDir = `${reposFilesDir}-test`;
	const reposFilesExpectedDir = `${reposFilesDir}-expected`;

	function toDep(
		repoUrl: string,
		type: "pr/mr" | "branch",
		reference: string | number
	): dep.PrDependency {
		return { repoUrl: repoUrl, type: type, reference: reference };
	}

	it("should not do anything if not a PR", async () => {
		let payload = {};
		expect(dep.getPrDependencies(payload)).toEqual([]);
		payload = { pull_request: {} };
		expect(dep.getPrDependencies(payload)).toEqual([]);
		payload = { pull_request: { body: "" } };
		expect(dep.getPrDependencies(payload)).toEqual([]);
	});

	it("should extract dependencies from the PR body", async () => {
		const bodyEmpty = `
Description of the changes.

Blah blah.
`;
		let payload = {};
		payload = { pull_request: { body: bodyEmpty } };
		expect(dep.getPrDependencies(payload)).toEqual([]);

		const body = `
Description of the changes.

Blah blah.

action-ros-ci-dependency: https://github.com/user/repo/pull/123
action-ros-ci-dependency : https://github.com/user/repo/pull/123
	 action-ros-ci-dependency:
action-ros-ci-dependency: 
  action-ros-ci-dependency: https://github.com/other-user/other-repo/pull/456/
  action-ros-ci-dependency: https://github.com/other-user/other-repo/pull
action-ros-ci-dependency: http://github.com/some-user/some_repo/pull/78
action-ros-ci-dependency: http://github.com/some-user/some_repo/tree/some-user/some-branch2
action-ros-ci-dependency: https://gitlab.com/some-user/some_repo/-/merge_requests/9  
action-ros-ci-dependency: https://gitlab.com/some-user/some_repo/-/tree/user/awesome_branch  
action-ros-ci-dependency: github.com/some-user/some-repo/pull/9
action-ros-ci-dependency: http://gitlab.com/org/some-project/some_repo/-/merge_requests/1
`;
		payload = { pull_request: { body: body } };
		const expected: dep.PrDependency[] = [
			toDep("https://github.com/user/repo", "pr/mr", 123),
			toDep("https://github.com/other-user/other-repo", "pr/mr", 456),
			toDep("http://github.com/some-user/some_repo", "pr/mr", 78),
			toDep("https://gitlab.com/some-user/some_repo", "pr/mr", 9),
			toDep("http://gitlab.com/org/some-project/some_repo", "pr/mr", 1),
			toDep(
				"http://github.com/some-user/some_repo",
				"branch",
				"some-user/some-branch2"
			),
			toDep(
				"https://gitlab.com/some-user/some_repo",
				"branch",
				"user/awesome_branch"
			),
		];
		// Order matters for equality here, otherwise it doesn't really matter
		expect(dep.getPrDependencies(payload)).toEqual(expected);
	});

	async function prepareTestFiles(): Promise<string[]> {
		if (fs.existsSync(reposFilesTestDir)) {
			fs.rmdirSync(reposFilesTestDir, { recursive: true });
		}
		// Copy input files for them to be (possibly) modified
		await io.cp(reposFilesDir, reposFilesTestDir, {
			force: true,
			recursive: true,
		});
		return fs.readdirSync(reposFilesTestDir);
	}

	function checkTestReposFiles(
		fileNames: string[],
		testDir: string,
		expectedDir: string
	): void {
		for (const filename of fileNames) {
			expect(filename.length).toBeGreaterThan(0);
			const result = yaml.safeLoad(
				fs.readFileSync(path.join(testDir, filename), "utf8")
			);
			const expected = yaml.safeLoad(
				fs.readFileSync(path.join(expectedDir, filename), "utf8")
			);
			expect(result).toEqual(expected);
		}
	}

	it("should work fine if there are no repos files and no dependencies", async () => {
		expect(
			dep.replaceDependencyVersions([], reposFilesTestDir, [])
		).toBeUndefined();
	});

	it("should not modify repos files if there are no dependencies", async () => {
		// Try to replace versions without dependencies
		const filenames = await prepareTestFiles();
		const filesPaths = filenames.map((f) => path.join(reposFilesTestDir, f));
		expect(
			dep.replaceDependencyVersions(filesPaths, reposFilesTestDir, [])
		).toBeUndefined();

		// Compare test files to original files
		expect(filesPaths.length).toEqual(2);
		expect(filenames.length).toEqual(2);
		checkTestReposFiles(filenames, reposFilesTestDir, reposFilesDir);
	});

	it("should replace versions of dependencies in repos files and create an extra repos file", async () => {
		const dependencies: dep.PrDependency[] = [
			toDep("https://github.com/ros2/launch", "pr/mr", 460),
			toDep("https://github.com/ros2/rcpputils", "branch", "h-turtle"),
			toDep("https://github.com/ros2/rcl_logging", "pr/mr", 53),
			toDep("http://github.com/ros2/rclcpp", "pr/mr", 1500),
			toDep("https://github.com/ros-tooling/action-ros-ci", "pr/mr", 490),
			toDep(
				"https://gitlab.com/ros-tracing/tracetools_analysis",
				"branch",
				"an/awesome-feature_branch2"
			),
			toDep("https://gitlab.com/ros-tracing/ros2_tracing", "pr/mr", 219),
		];

		// Replace versions
		const filenames = await prepareTestFiles();
		const filesPaths = filenames.map((f) => path.join(reposFilesTestDir, f));
		const replaceRet = dep.replaceDependencyVersions(
			filesPaths,
			reposFilesTestDir,
			dependencies
		);
		expect(replaceRet).toEqual(path.join(reposFilesTestDir, "extra.repos"));

		// Compare test files to expected files
		filenames.push(path.basename(replaceRet || ""));
		expect(filesPaths.length).toEqual(2);
		expect(filenames.length).toEqual(3);
		checkTestReposFiles(filenames, reposFilesTestDir, reposFilesExpectedDir);
	});

	it("should create an extra repos file for all dependencies if no repos files are given", async () => {
		const dependencies: dep.PrDependency[] = [
			toDep("https://github.com/ros-tooling/action-ros-ci", "pr/mr", 490),
			toDep(
				"https://gitlab.com/ros-tracing/tracetools_analysis",
				"branch",
				"an/awesome-feature_branch2"
			),
		];

		// Replace versions
		await prepareTestFiles();
		const replaceRet = dep.replaceDependencyVersions(
			[],
			reposFilesTestDir,
			dependencies
		);
		expect(replaceRet).toEqual(path.join(reposFilesTestDir, "extra.repos"));

		// Compare test files to expected files
		const filenames = [path.basename(replaceRet || "")];
		checkTestReposFiles(filenames, reposFilesTestDir, reposFilesExpectedDir);
	});

	it("should create full URLs from dependencies", async () => {
		expect(
			dep.getFullUrlFromDependency(
				toDep("https://github.com/some-user/some_repo", "pr/mr", 123)
			)
		).toEqual("https://github.com/some-user/some_repo/pull/123");
		expect(
			dep.getFullUrlFromDependency(
				toDep("https://gitlab.com/some-user/some_repo", "pr/mr", 123)
			)
		).toEqual("https://gitlab.com/some-user/some_repo/-/merge_requests/123");
		expect(
			dep.getFullUrlFromDependency(
				toDep(
					"https://github.com/some-user/some_repo",
					"branch",
					"my/awesome-feature_branch42"
				)
			)
		).toEqual(
			"https://github.com/some-user/some_repo/tree/my/awesome-feature_branch42"
		);
		expect(
			dep.getFullUrlFromDependency(
				toDep(
					"https://gitlab.com/some-user/some_repo",
					"branch",
					"my/awesome-feature_branch42"
				)
			)
		).toEqual(
			"https://gitlab.com/some-user/some_repo/-/tree/my/awesome-feature_branch42"
		);
	});
});
