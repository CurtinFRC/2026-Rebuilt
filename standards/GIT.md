# Team 4788 Git Standards

## Commits
- prefix with category either fix, chore or feature. prefixes should be the prefix name followed by a colon, ie `fix: message here`
- fixs are small bug fixes or adjustments that improve the code
- featurs are new features you are adding to the code
- chores are small tasks or refactors that are done to the code that aren't necessarily fixing or adding something but instead do things like update versions or refactor for clarity
- commits to main will be squashed from a pull request
- your commit message should contain a description of what the commit does for example `Add shooter subsystem`
- a full commit might be for example, `chore: update dependencies to latest version`
- commits can also be used to close an issue, if you do this then add `Closes #issue number` to the description

## Pull Requests
- each pull request should encapsulate exactly one thing, ie add a shooter subsystem
- your pull request description should clearly tell the reader what the pull request does and add any information required to understand it
- if your pull request resolves an issue you should add `Closes #issue number` to the description
- the pull request title should be named like a commit describing the aim of the whole pull request

## Branches
- a branch is where you collect your commits to make a pull request
- branches should always be split off from the main branch, not another branch, except in some cases such as if you are pull requesting a change to someone elses branch
- branches should be named name/branch-task, for example `jade/add-intake-subsystem`, or named name/issue for example `jade/19`

## Checks
- before merging into the main branch you need to pass all of our checks
- first you need to pass the automated checks done using github actions, if the formatting fails then run `./gradlew spotlessApply` and commit the changes
- second you need to pass through code review from mentors or programming leadership
- other students may also review your code
- finally you need to make sure to test your code, ideally on a real robot but in sim or replay can also be fine depending on the code
