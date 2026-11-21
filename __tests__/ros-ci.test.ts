import * as core from "@actions/core";
import * as tr from "@actions/exec/lib/toolrunner";
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
		expect(actionRosCi.validateDistros("", "kilted")).toBe(true);
		expect(actionRosCi.validateDistros("", "lyrical")).toBe(true);
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

describe("execShellCommand error reporting on Windows", () => {
	const platformDescriptor = Object.getOwnPropertyDescriptor(
		process,
		"platform",
	) as PropertyDescriptor;
	const setPlatform = (platform: string): void => {
		Object.defineProperty(process, "platform", {
			...platformDescriptor,
			value: platform,
		});
	};
	afterEach(() => {
		Object.defineProperty(process, "platform", platformDescriptor);
		jest.restoreAllMocks();
	});

	it("surfaces the failing command, the original cmd.exe error and runner info when a step fails on Windows", async () => {
		setPlatform("win32");
		const originalError = new Error(
			"The process 'C:\\Windows\\system32\\cmd.exe' failed with exit code 1",
		);
		const execSpy = jest
			.spyOn(tr.ToolRunner.prototype, "exec")
			.mockRejectedValue(originalError);
		let caught: unknown;
		try {
			await execShellCommand(["colcon", "build", "--merge-install"]);
		} catch (error) {
			caught = error;
		}
		expect(execSpy).toHaveBeenCalled();
		expect(caught).toBeInstanceOf(Error);
		const message = (caught as Error).message;
		// the full Windows tool runner command line, incl. vcvars64.bat and git bash
		expect(message).toContain("cmd.exe");
		expect(message).toContain("vcvars64.bat");
		expect(message).toContain("bash.exe");
		// the actual command that failed
		expect(message).toContain("colcon build --merge-install");
		// the underlying error is preserved
		expect(message).toContain(originalError.message);
		// runner info is included
		expect(message).toContain("platform: win32");
		// the original error is kept as cause
		expect((caught as Error & { cause?: unknown }).cause).toBe(originalError);
	});

	it("keeps the command as separate arguments on Windows when bash is not forced", async () => {
		setPlatform("win32");
		jest
			.spyOn(tr.ToolRunner.prototype, "exec")
			.mockRejectedValue(
				new Error("The process 'cmd.exe' failed with exit code 2"),
			);
		let caught: unknown;
		try {
			await execShellCommand(["colcon", "test"], undefined, false);
		} catch (error) {
			caught = error;
		}
		expect(caught).toBeInstanceOf(Error);
		const message = (caught as Error).message;
		expect(message).toContain("cmd.exe");
		expect(message).toContain("colcon test");
		expect(message).toContain("failed with exit code 2");
		// no git bash in the command line when bash is not forced
		expect(message).not.toContain("bash.exe");
	});

	it("still returns the exit code without throwing when the command succeeds", async () => {
		setPlatform("win32");
		jest.spyOn(tr.ToolRunner.prototype, "exec").mockResolvedValue(0);
		await expect(execShellCommand(["colcon", "build"])).resolves.toBe(0);
	});
});

describe("execShellCommand error reporting on non-Windows platforms", () => {
	afterEach(() => {
		jest.restoreAllMocks();
	});

	it("surfaces the failing command and the original error", async () => {
		const originalError = new Error(
			"The process '/bin/bash' failed with exit code 2",
		);
		jest
			.spyOn(tr.ToolRunner.prototype, "exec")
			.mockRejectedValue(originalError);
		let caught: unknown;
		try {
			await execShellCommand(["somefailingcommand", "--flag"]);
		} catch (error) {
			caught = error;
		}
		expect(caught).toBeInstanceOf(Error);
		const message = (caught as Error).message;
		expect(message).toContain("bash");
		expect(message).toContain("somefailingcommand --flag");
		expect(message).toContain(originalError.message);
		expect(message).toContain(`platform: ${process.platform}`);
		expect((caught as Error & { cause?: unknown }).cause).toBe(originalError);
	});

	it("reports failures even when the command is run silently", async () => {
		const groupSpy = jest.spyOn(core, "group");
		jest
			.spyOn(tr.ToolRunner.prototype, "exec")
			.mockRejectedValue(new Error("silent failure"));
		let caught: unknown;
		try {
			await execShellCommand(["secret-command"], { silent: true });
		} catch (error) {
			caught = error;
		}
		expect(groupSpy).not.toHaveBeenCalled();
		expect(caught).toBeInstanceOf(Error);
		expect((caught as Error).message).toContain("secret-command");
		expect((caught as Error).message).toContain("silent failure");
	});
});

describe("enrichExecError", () => {
	it("includes the command line, runner info and the underlying error, and keeps the cause", () => {
		const original = new Error("The process 'cmd.exe' failed with exit code 1");
		const enriched = actionRosCi.enrichExecError(
			original,
			"cmd.exe /C call vcvars64.bat && colcon build",
		);
		expect(enriched.message).toContain("colcon build");
		expect(enriched.message).toContain("vcvars64.bat");
		expect(enriched.message).toContain(original.message);
		expect(enriched.message).toContain("platform:");
		expect(enriched.message).toContain(process.platform);
		expect((enriched as Error & { cause?: unknown }).cause).toBe(original);
		expect(enriched.stack).toContain(original.message);
	});

	it("handles non-Error thrown values", () => {
		const enriched = actionRosCi.enrichExecError("plain failure", "some cmd");
		expect(enriched.message).toContain("plain failure");
		expect(enriched.message).toContain("some cmd");
	});
});

describe("reportError", () => {
	afterEach(() => {
		jest.restoreAllMocks();
	});

	it("fails the action with the real error message and logs the stack trace", () => {
		const setFailedSpy = jest
			.spyOn(core, "setFailed")
			.mockImplementation(() => {});
		const consoleErrorSpy = jest
			.spyOn(console, "error")
			.mockImplementation(() => {});
		const error = new Error("detailed failure message");
		actionRosCi.reportError(error);
		expect(setFailedSpy).toHaveBeenCalledWith("detailed failure message");
		expect(consoleErrorSpy).toHaveBeenCalledWith(error.stack);
	});

	it("reports non-Error thrown values instead of 'Unknown error'", () => {
		const setFailedSpy = jest
			.spyOn(core, "setFailed")
			.mockImplementation(() => {});
		actionRosCi.reportError("plain string failure");
		expect(setFailedSpy).toHaveBeenCalledWith("plain string failure");
	});

	it("falls back to 'Unknown error' for nullish values", () => {
		const setFailedSpy = jest
			.spyOn(core, "setFailed")
			.mockImplementation(() => {});
		actionRosCi.reportError(undefined);
		expect(setFailedSpy).toHaveBeenCalledWith("Unknown error");
	});
});
