import * as core from "@actions/core";
import * as path from "path";
import * as yaml from "js-yaml";
import fs from "fs";
import { WebhookPayload } from "@actions/github/lib/interfaces";

// Expecting something like:
//  for PRs/MRs:
//        action-ros-ci-dependency: https://github.com/user/some-repo/pull/123
//    or
//        action-ros-ci-dependency: https://gitlab.com/user/some-repo/-/merge_requests/123
const REGEX_DEPENDENCY_PR_GITHUB = /action-ros-ci-dependency: (http[s]?:\/\/github.com\/[\w-]+\/[\w-]+)\/pull\/(\d+)/g;
const REGEX_DEPENDENCY_MR_GITLAB = /action-ros-ci-dependency: (http[s]?:\/\/gitlab.com\/[\w-]+(?:\/[\w-]+)+)\/-\/merge_requests\/(\d+)/g;
//  for branches:
//        action-ros-ci-dependency: https://github.com/user/some-repo/tree/some-branch
//    or
//        action-ros-ci-dependency: https://gitlab.com/user/some-repo/-/tree/some-branch
const REGEX_DEPENDENCY_BRANCH_GITHUB = /action-ros-ci-dependency: (http[s]?:\/\/github.com\/[\w-]+\/[\w-]+)\/tree\/([\w-/]+)/g;
const REGEX_DEPENDENCY_BRANCH_GITLAB = /action-ros-ci-dependency: (http[s]?:\/\/gitlab.com\/[\w-]+(?:\/[\w-]+)+)\/-\/tree\/([\w-/]+)/g;

const REGEX_URL_LEADING = /^http[s]?:\/\//;
const REGEX_URL_TRAILING = /\/?(?:.git)?\/?$/;

/**
 * Extract captures of all matches.
 *
 * The first dimension contains all the matches and the second
 * dimension contains the captures for the given match.
 *
 * @param content the string in which to search
 * @param regex the regex to apply
 * @returns the array of all captures for all matches
 */
function extractCaptures(content: string, regex: RegExp): string[][] {
	const captures: string[][] = [];
	let matches;
	while ((matches = regex.exec(content)) !== null) {
		const capture = matches.slice(1);
		if (capture.length > 0) {
			captures.push(capture);
		}
	}
	return captures;
}

/**
 * Container for PR dependency info.
 */
export interface PrDependency {
	/**
	 * The URL to the repo.
	 */
	repoUrl: string;
	/**
	 * The type of dependency.
	 * It's either a PR/MR or a branch.
	 */
	type: "pr/mr" | "branch";
	/**
	 * The reference, with a meaning that depends on
	 * the type (PR/MR number or branch name string).
	 */
	reference: number | string;
}

/**
 * Get PR dependencies.
 *
 * @param contextPayload the github context payload object
 * @returns an unordered array with all declared dependencies
 */
export function getPrDependencies(
	contextPayload: WebhookPayload
): PrDependency[] {
	const prPayload = contextPayload.pull_request;
	if (!prPayload) {
		return [];
	}
	const prBody = prPayload.body;
	if (!prBody) {
		return [];
	}

	const prMrDependencies = [
		...extractCaptures(prBody, REGEX_DEPENDENCY_PR_GITHUB),
		...extractCaptures(prBody, REGEX_DEPENDENCY_MR_GITLAB),
	]
		.filter((capture) => capture.length === 2)
		.map((capture) => {
			return {
				repoUrl: capture[0],
				type: "pr/mr",
				reference: +capture[1],
			} as PrDependency;
		});
	const branchDependencies = [
		...extractCaptures(prBody, REGEX_DEPENDENCY_BRANCH_GITHUB),
		...extractCaptures(prBody, REGEX_DEPENDENCY_BRANCH_GITLAB),
	]
		.filter((capture) => capture.length === 2)
		.map((capture) => {
			return {
				repoUrl: capture[0],
				type: "branch",
				reference: capture[1],
			} as PrDependency;
		});
	return [...prMrDependencies, ...branchDependencies];
}

/**
 * Compare two git remotes/URLs.
 *
 * Removes leading & trailing superfluous elements so that it can
 * properly compare URLs that may not otherwise match as strings.
 *
 * @param left the left URL
 * @param right the right URL
 * @returns true if URLs are equal, false otherwise
 */
function areUrlsEqual(left: string, right: string): boolean {
	const mainUrlLeft = left
		.replace(REGEX_URL_LEADING, "")
		.replace(REGEX_URL_TRAILING, "");
	const mainUrlRight = right
		.replace(REGEX_URL_LEADING, "")
		.replace(REGEX_URL_TRAILING, "");
	return mainUrlLeft === mainUrlRight;
}

/**
 * Compare a possible superset array to a reference array and
 * return the (relative) complement of the reference array.
 *
 * @param ref the reference array
 * @param superset the possible superset array to be compared to the reference
 * @returns the array of elements that are in the superset but not in the reference array
 */
function supersetDiff<T>(ref: T[], superset: T[]): T[] {
	return superset.filter((eSuper) => !ref.includes(eSuper));
}

/**
 * Get git ref for given dependency.
 *
 * @param dependency the dependency
 * @returns the ref string
 */
function getRefFromDependency(dependency: PrDependency): string {
	if (dependency.type === "pr/mr") {
		// Note: GitLab does not seem to always support refs to merged MRs, so we use the MR's HEAD
		return dependency.repoUrl.includes("gitlab.com")
			? `merge-requests/${dependency.reference}`
			: `pull/${dependency.reference}/merge`;
	} else {
		// Simple branch
		return String(dependency.reference);
	}
}

/**
 * Replace versions of repo dependencies in given repos files, and
 * create an extra repos files with all dependencies that were not in the given repos files.
 *
 * @param reposFiles the full paths to repos files
 * @param reposFilesDir the full path to the directory that should contain the repos files
 * @param dependencies the declared dependencies
 * @returns the full path to an extra repos file if one was created, otherwise `undefined`
 */
export function replaceDependencyVersions(
	reposFiles: string[],
	reposFilesDir: string,
	dependencies: PrDependency[]
): string | undefined {
	const matchedDependencies: PrDependency[] = [];
	for (const reposFile of reposFiles) {
		const content = yaml.safeLoad(fs.readFileSync(reposFile, "utf8"));
		const repos = content["repositories"];
		let fileModified = false;
		for (const repoPath in repos) {
			// Check if a dependency was declared for this repo
			const repoInfo = repos[repoPath];
			const matchingDependencies = dependencies.filter((dependency) => {
				return areUrlsEqual(dependency.repoUrl, repoInfo["url"]);
			});
			if (matchingDependencies.length >= 1) {
				fileModified = true;
				const matchedDependency = matchingDependencies[0];
				matchedDependencies.push(matchedDependency);
				// Replace version with ref pointing to PR/MR depending on the website
				repoInfo["version"] = getRefFromDependency(matchedDependency);
				if (matchingDependencies.length > 1) {
					core.error(`more than 1 dependency match for: ${repoPath}`);
				}
			}
		}
		// Overwrite file if content was modified
		if (fileModified) {
			fs.writeFileSync(reposFile, yaml.safeDump(content));
		}
	}
	// Figure out which dependencies (if any) were not found in repos files
	const unmatchedDependencies = supersetDiff(matchedDependencies, dependencies);
	if (unmatchedDependencies.length === 0) {
		return undefined;
	}
	// Create an extra repos file with all unmatched dependencies
	const extraRepos = { repositories: {} };
	for (const [index, unmatchedDependency] of unmatchedDependencies.entries()) {
		extraRepos["repositories"][`extra/${index}`] = {
			type: "git",
			url: unmatchedDependency.repoUrl,
			version: getRefFromDependency(unmatchedDependency),
		};
	}
	const extraReposFile = path.join(reposFilesDir, "extra.repos");
	fs.writeFileSync(extraReposFile, yaml.safeDump(extraRepos));
	return extraReposFile;
}

/**
 * Reconstruct full URL from dependency.
 *
 * @param dependency the dependency
 * @returns the full URL
 */
export function getFullUrlFromDependency(dependency: PrDependency): string {
	const isGitlab = dependency.repoUrl.includes("gitlab.com");
	if (dependency.type === "pr/mr") {
		// PR or MR
		return isGitlab
			? `${dependency.repoUrl}/-/merge_requests/${dependency.reference}`
			: `${dependency.repoUrl}/pull/${dependency.reference}`;
	} else {
		// Branch
		return isGitlab
			? `${dependency.repoUrl}/-/tree/${dependency.reference}`
			: `${dependency.repoUrl}/tree/${dependency.reference}`;
	}
}
