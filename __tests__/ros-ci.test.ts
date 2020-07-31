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
