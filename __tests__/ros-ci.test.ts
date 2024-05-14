import * as core from "@actions/core";
import * as actionRosCi from "../src/action-ros-ci";
import * as dep from "../src/dependencies";
import { execShellCommand } from "../src/action-ros-ci";

jest.setTimeout(20000); // in milliseconds

describe("execShellCommand test suite", () => {
	it("calls coreGroup", async () => {
		const mockGroup = jest.spyOn(core, "group");
		const result = await execShellCommand(["echo Hello World"]);
		expect(mockGroup).toHaveBeenCalled();
		expect(result).toEqual(0);
	});
	it("uses a prefix", async () => {
		const mockGroup = jest.spyOn(core, "group");
		const result = await execShellCommand(["echo", "Hello World"]);
		expect(mockGroup).toHaveBeenCalled();
		expect(result).toEqual(0);
	});
	it("ignores return code", async () => {
		const mockGroup = jest.spyOn(core, "group");
		const options = {
			ignoreReturnCode: true,
		};
		const result = execShellCommand(["somebadcommand"], options);
		expect(mockGroup).toHaveBeenCalled();
		expect(result).not.toEqual(0);
	});
});

describe("validate distribution test", () => {
	it("validates that the ros distribution validator acts correctly", async () => {
		expect(actionRosCi.validateDistros("kinetic", "")).toBe(true);
		expect(actionRosCi.validateDistros("lunar", "")).toBe(true);
		expect(actionRosCi.validateDistros("melodic", "")).toBe(true);
		expect(actionRosCi.validateDistros("noetic", "")).toBe(true);
		expect(actionRosCi.validateDistros("", "dashing")).toBe(true);
		expect(actionRosCi.validateDistros("", "eloquent")).toBe(true);
		expect(actionRosCi.validateDistros("", "foxy")).toBe(true);
		expect(actionRosCi.validateDistros("", "galactic")).toBe(true);
		expect(actionRosCi.validateDistros("", "humble")).toBe(true);
		expect(actionRosCi.validateDistros("", "iron")).toBe(true);
		expect(actionRosCi.validateDistros("", "jazzy")).toBe(true);
		expect(actionRosCi.validateDistros("", "rolling")).toBe(true);

		expect(actionRosCi.validateDistros("noetic", "rolling")).toBe(true);
		expect(actionRosCi.validateDistros("melodic", "dashing")).toBe(true);
		expect(actionRosCi.validateDistros("noetic", "humble")).toBe(true);

		expect(actionRosCi.validateDistros("", "")).toBe(false);
		expect(actionRosCi.validateDistros("groovy", "")).toBe(false);
		expect(actionRosCi.validateDistros("", "bouncy")).toBe(false);
		expect(actionRosCi.validateDistros("apples", "bananas")).toBe(false);
		expect(actionRosCi.validateDistros("apples", "rolling")).toBe(false);
		// Check and reset process exit code, which is set when core.setFailed() is called
		expect(process.exitCode).toBe(core.ExitCode.Failure);
		process.exitCode = core.ExitCode.Success;
	});
});

describe("utilities", () => {
	it("should join and filter out non empty elements", () => {
		expect(actionRosCi.filterNonEmptyJoin([])).toBe("");
		expect(actionRosCi.filterNonEmptyJoin([""])).toBe("");
		expect(actionRosCi.filterNonEmptyJoin(["", ""])).toBe("");
		expect(actionRosCi.filterNonEmptyJoin(["abc"])).toBe("abc");
		expect(actionRosCi.filterNonEmptyJoin(["abc", "def"])).toBe("abc def");
		expect(actionRosCi.filterNonEmptyJoin(["abc", "def", ""])).toBe("abc def");
		expect(actionRosCi.filterNonEmptyJoin(["", "abc", "", "", "def"])).toBe(
			"abc def",
		);
	});
});

describe("PR-specific repos files", () => {
	it("should not do anything if not a PR", async () => {
		let payload = {};
		expect(dep.getReposFilesOverride(payload)).toEqual([]);
		expect(dep.getReposFilesSupplemental(payload)).toEqual([]);
		payload = { pull_request: {} };
		expect(dep.getReposFilesOverride(payload)).toEqual([]);
		expect(dep.getReposFilesSupplemental(payload)).toEqual([]);
		payload = { pull_request: { body: "" } };
		expect(dep.getReposFilesOverride(payload)).toEqual([]);
		expect(dep.getReposFilesSupplemental(payload)).toEqual([]);
	});

	it("should extract repos files from the PR body", () => {
		const bodyEmpty = `
Description of the changes.
Blah blah.
`;
		let payload = {};
		payload = { pull_request: { body: bodyEmpty } };
		expect(dep.getReposFilesOverride(payload)).toEqual([]);
		expect(dep.getReposFilesSupplemental(payload)).toEqual([]);

		const body = `
Description of the changes.
Blah blah.

action-ros-ci-repos-override:   
action-ros-ci-repos-override: https://raw.githubusercontent.com/ros2/ros2/rolling/ros2.repos
action-ros-ci-repos-override : https://some.website.repos
 action-ros-ci-repos-override:  https://gist.github.com/some-user/some-gist.repos
 action-ros-ci-repos-supplemental:https://gist.github.com/some-user/some-other-gist.repos
action-ros-ci-repos-supplemental:  file://path/to/some/file.txt 
`;
		payload = { pull_request: { body: body } };
		const expectedOverride = [
			"https://raw.githubusercontent.com/ros2/ros2/rolling/ros2.repos",
			"https://gist.github.com/some-user/some-gist.repos",
		];
		const expectedSupplemental = [
			"https://gist.github.com/some-user/some-other-gist.repos",
			"file://path/to/some/file.txt",
		];
		expect(dep.getReposFilesOverride(payload)).toEqual(
			expect.arrayContaining(expectedOverride),
		);
		expect(dep.getReposFilesSupplemental(payload)).toEqual(
			expect.arrayContaining(expectedSupplemental),
		);
	});
});
