import * as core from '@actions/core'
import * as actionRosCi from "../src/action-ros-ci"
import {execBashCommand} from '../src/action-ros-ci'

jest.setTimeout(20000);  // in milliseconds

describe('execBashCommand test suite', () => {
  it('calls coreGroup', async () => {
      const mockGroup = jest.spyOn(core, 'group');
      const result = await execBashCommand('echo "Hello World"');
      expect(mockGroup).toBeCalled();
      expect(result).toEqual(0);
  });
  it('uses a prefix', async () => {
    const mockGroup = jest.spyOn(core, 'group');
    const result = await execBashCommand('Hello World', 'echo ');
    expect(mockGroup).toBeCalled();
    expect(result).toEqual(0);
  });
  it('ignores return code', async () => {
    const mockGroup = jest.spyOn(core, 'group');
    const options = {
      ignoreReturnCode: true
    }
    const result = execBashCommand('somebadcommand', '', options);
    expect(mockGroup).toBeCalled();
    expect(result).not.toEqual(0);
  })
})

describe("validate distribution test", () => {
	it("test valid", async () => {
		await expect(actionRosCi.validateDistro("kinetic", "", "")).toBe(true);
		await expect(actionRosCi.validateDistro("lunar", "", "")).toBe(true);
		await expect(actionRosCi.validateDistro("melodic", "", "")).toBe(true);
		await expect(actionRosCi.validateDistro("noetic", "", "")).toBe(true);
		await expect(actionRosCi.validateDistro("", "dashing", "")).toBe(true);
		await expect(actionRosCi.validateDistro("", "eloquent", "")).toBe(true);
		await expect(actionRosCi.validateDistro("", "foxy", "")).toBe(true);
	});

	it("test not valid", async () => {
		//ROS1 End-of-Life
		await expect(actionRosCi.validateDistro("box", "", "")).toBe(false);
		await expect(actionRosCi.validateDistro("c", "", "")).toBe(false);
		await expect(actionRosCi.validateDistro("diamondback", "", "")).toBe(false);
		await expect(actionRosCi.validateDistro("electric", "", "")).toBe(false);
		await expect(actionRosCi.validateDistro("fuerte", "", "")).toBe(false);
		await expect(actionRosCi.validateDistro("groovy", "", "")).toBe(false);
		await expect(actionRosCi.validateDistro("hydro", "", "")).toBe(false);
		await expect(actionRosCi.validateDistro("indigo", "", "")).toBe(false);
		await expect(actionRosCi.validateDistro("jade", "", "")).toBe(false);
		//ROS2 End-of-Life
		await expect(actionRosCi.validateDistro("", "ardent", "")).toBe(false);
		await expect(actionRosCi.validateDistro("", "bouncy", "")).toBe(false);
		await expect(actionRosCi.validateDistro("", "crystal", "")).toBe(false);
	});
});
